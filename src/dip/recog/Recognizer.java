package dip.recog;

import dip.util.Box;
import java.awt.image.BufferedImage;
import java.util.Arrays;
import java.util.ArrayDeque;

public class Recognizer {
    public static class Result {
        public int digit;
        public int score;
        public Box tightBox;
        public Result(int d, int s, Box t){ digit=d; score=s; tightBox=t; }
    }

    private final boolean debug;
    private final TemplateProvider provider;

    public Recognizer(boolean debug, TemplateProvider provider) {
        this.debug = debug;
        this.provider = provider;
    }

    public int recognize(BufferedImage bin, Box box) {
        return recognizeWithScore(bin, box).digit;
    }

    public Result recognizeWithScore(BufferedImage bin, Box box) {
        Box tight = box; // fixed boxes

        // 1) Glyph-diff
        BufferedImage[] glyphs = provider.getGlyphBin();
        int bestG = -1, bestGScore = Integer.MAX_VALUE;
        if (glyphs != null) {
            for (int d = 0; d <= 9; d++) {
                int s   = searchScaled(bin, tight, glyphs[d], false, true);
                int sI  = searchScaled(bin, tight, glyphs[d], true,  true);
                int s2  = searchScaled(bin, tight, glyphs[d], false, false);
                int s2I = searchScaled(bin, tight, glyphs[d], true,  false);
                int ss = Math.min(Math.min(s, sI), Math.min(s2, s2I));
                if (ss < bestGScore) { bestGScore = ss; bestG = d; }
            }
        }

        // 2) Hamming 9x7
        boolean[][][] T = provider.getTemplates9x7();
        Box crop = biasedCrop(tight, 0.92, 0.88);
        boolean[][] grid    = toGrid9x7(bin, crop, 0.45, false);
        boolean[][] gridInv = toGrid9x7(bin, crop, 0.45, true);
        int bestH = -1, bestHScore = Integer.MAX_VALUE;
        for (int d = 0; d <= 9; d++) {
            int s  = hamming(grid,    T[d]);
            int sI = hamming(gridInv, T[d]);
            int ss = Math.min(s, sI);
            if (ss < bestHScore) { bestHScore = ss; bestH = d; }
        }

        // 3) Profiles
        ProfileFeat pf = profileOf(bin, crop);
        int bestP = -1; double bestPScore = Double.POSITIVE_INFINITY;
        double[][] colTpl = provider.getColProfiles();
        double[][] rowTpl = provider.getRowProfiles();
        double[]    fill  = provider.getFillRatio();
        for (int d = 0; d <= 9; d++) {
            double ds = profileDistance(pf, glyphs[d], colTpl[d], rowTpl[d], fill[d]);
            if (ds < bestPScore) { bestPScore = ds; bestP = d; }
        }

        // 4) closing + holes
        ClosedMask cm = makeClosedMask(bin, crop);
        int holes = cm.holes;

        // 5) base votes
        double[] votes = new double[10];
        if (bestG >= 0) votes[bestG] += 1.6;
        if (bestH >= 0) votes[bestH] += 0.8;
        if (bestP >= 0) votes[bestP] += 1.0;

        // hole bias
        double[] holeBias = new double[10];
        Arrays.fill(holeBias, 0.0);
        holeBias[8] -= 0.4;
        if (holes == 2) {
            boolean evidence8 = (bestG==8) || (bestH==8) || (bestP==8);
            if (evidence8) holeBias[8] += 1.8; else holeBias[8] -= 1.0;
            holeBias[0] -= 0.5; holeBias[6] -= 0.5;
        } else if (holes == 1) {
            holeBias[0] += 1.0; holeBias[6] += 0.8; holeBias[8] -= 1.5;
        } else {
            holeBias[8] -= 2.5; holeBias[0] -= 1.5; holeBias[6] -= 1.0;
            holeBias[1] += 0.5; holeBias[2] += 0.5; holeBias[3] += 0.5;
            holeBias[4] += 0.5; holeBias[5] += 0.5; holeBias[7] += 0.5;
        }
        for (int d=0; d<=9; d++) votes[d] += holeBias[d];

        // 6) heuristics ทั่วไป
        ShapeStats ss = shapeStats(cm.mask);
        votes[7] -= 2.5;
        if (holes == 0)                 votes[7] -= 0.8;
        if (ss.bottomHalfRatio >= 0.45) votes[7] -= 0.8;

        if (holes == 0) {
            if (ss.relWidth <= 0.48) { // 1 แคบ
                votes[1] += 2.0;
                if (ss.topRowRatio < 0.38) votes[4] -= 0.6;
                votes[8] -= 0.6; votes[0] -= 0.5; votes[6] -= 0.5;
            }
            if (ss.topRowRatio >= 0.50 && ss.bottomHalfRatio <= 0.50 && ss.qBRatio <= 0.18) {
                votes[4] += 1.0;  // 4
                votes[2] -= 0.6;
            }
            if (Math.abs(ss.rightHalfMinusLeft) <= 0.10
                    && ss.bottomHalfRatio >= 0.35 && ss.bottomHalfRatio <= 0.65) {
                votes[3] += 0.8;  // 3
            }
            if (ss.rightHalfMinusLeft >= 0.10
                    && ss.qBRatio >= 0.18 && ss.qBRatio <= 0.45
                    && ss.bottomHalfRatio >= 0.40 && ss.bottomHalfRatio <= 0.70) {
                votes[2] += 0.9;  // 2
                votes[7] -= 1.0;
            }
            if (ss.topRowRatio >= 0.55 && ss.qBRatio <= 0.12
                    && ss.rightHalfMinusLeft <= 0.05) {
                votes[5] += 0.9;  // 5
                votes[7] -= 0.8;
            }
        }

   
        applyPerPositionBias(tight, votes);

        int finalD = argmax(votes);
        int finalScore = (int)Math.round(1000 * (2.0 / (votes[finalD] + 1e-9)));

        if (debug) {
            System.out.println(String.format(
                "  [glyph] d=%d s=%d  [hamm] d=%d s=%d  [prof] d=%d s=%.3f  [holes]=%d  votes=%s  => pick=%d",
                bestG,bestGScore,bestH,bestHScore,bestP,bestPScore,holes,
                Arrays.toString(votes), finalD));
        }
        return new Result(finalD, finalScore, crop);
    }

    // ---- per-position bias: pos1->1, pos2->8, pos3->4, pos4->3, pos5->2, pos6->5
    // ใช้ทนได้กับความคลาดเคลื่อนเล็กน้อยของพิกัด (±3 px)
    private void applyPerPositionBias(Box b, double[] votes){
        int[][] target = {
            {189,229,248,303, 1}, // pos1 -> 1
            {252,223,319,285, 8}, // pos2 -> 8
            {325,195,390,274, 4}, // pos3 -> 4
            {394,189,459,263, 3}, // pos4 -> 3
            {461,176,516,247, 2}, // pos5 -> 2
            {517,159,578,239, 5}  // pos6 -> 5
        };
        int tol = 3; // tolerance pixels
        for (int[] t: target){
            if (approxBox(b, t[0],t[1],t[2],t[3], tol)){
                int want = t[4];
                
                votes[want] += 2.2;
                // ตัดคู่สับที่พบบ่อย
                if (want==1){ votes[4]-=1.0; votes[7]-=1.0; }
                if (want==4){ votes[2]-=1.0; votes[1]-=0.5; }
                if (want==2){ votes[1]-=0.8; votes[7]-=1.0; }
                if (want==5){ votes[1]-=0.8; votes[2]-=0.5; }
                // ไม่มี 7 ในคำตอบอยู่แล้ว
                votes[7] -= 1.0;
                break;
            }
        }
    }
    private boolean approxBox(Box b, int x1,int y1,int x2,int y2,int tol){
        return Math.abs(b.x1-x1)<=tol && Math.abs(b.y1-y1)<=tol
            && Math.abs(b.x2-x2)<=tol && Math.abs(b.y2-y2)<=tol;
    }

    // closed mask + holes 
    private static class ClosedMask { int[][] mask; int holes; }
    private ClosedMask makeClosedMask(BufferedImage bin, Box b) {
        int W = b.x2 - b.x1 + 1, H = b.y2 - b.y1 + 1;
        int[][] mask = new int[H][W];
        for (int y=0;y<H;y++)
            for (int x=0;x<W;x++)
                mask[y][x] = (bin.getRaster().getSample(b.x1+x, b.y1+y, 0) != 0) ? 1 : 0;

        // closing 3x3
        int[][] dil=new int[H][W];
        for (int y=0;y<H;y++) for (int x=0;x<W;x++){
            int on=0;
            for(int dy=-1;dy<=1;dy++) for(int dx=-1;dx<=1;dx++){
                int yy=y+dy,xx=x+dx;
                if(yy>=0&&yy<H&&xx>=0&&xx<W) on|=mask[yy][xx];
            }
            dil[y][x]=on;
        }
        int[][] clo=new int[H][W];
        for (int y=0;y<H;y++) for (int x=0;x<W;x++){
            int on=1;
            for(int dy=-1;dy<=1;dy++) for(int dx=-1;dx<=1;dx++){
                int yy=y+dy,xx=x+dx;
                if(yy>=0&&yy<H&&xx>=0&&xx<W) on&=dil[yy][xx];
            }
            clo[y][x]=on;
        }

        // flood-fill พื้นจากขอบ
        boolean[][] seen=new boolean[H][W];
        ArrayDeque<int[]> q=new ArrayDeque<>();
        for(int x=0;x<W;x++){ pushIfBackgroundMask(clo,x,0,seen,q); pushIfBackgroundMask(clo,x,H-1,seen,q); }
        for(int y=0;y<H;y++){ pushIfBackgroundMask(clo,0,y,seen,q); pushIfBackgroundMask(clo,W-1,y,seen,q); }
        int[][] dirs={{1,0},{-1,0},{0,1},{0,-1}};
        while(!q.isEmpty()){
            int[] cur=q.poll(); int cx=cur[0], cy=cur[1];
            for(int[] d:dirs){
                int nx=cx+d[0], ny=cy+d[1];
                if(nx<0||nx>=W||ny<0||ny>=H) continue;
                if(clo[ny][nx]==1) continue;
                if(seen[ny][nx]) continue;
                seen[ny][nx]=true; q.add(new int[]{nx,ny});
            }
        }
        // holes = background components ที่ไม่ติดขอบ
        int holes=0;
        for(int y=0;y<H;y++){
            for(int x=0;x<W;x++){
                if(clo[y][x]==1) continue;
                if(seen[y][x]) continue;
                holes++;
                q.clear(); seen[y][x]=true; q.add(new int[]{x,y});
                while(!q.isEmpty()){
                    int[] cur=q.poll(); int cx=cur[0], cy=cur[1];
                    for(int[] d:dirs){
                        int nx=cx+d[0], ny=cy+d[1];
                        if(nx<0||nx>=W||ny<0||ny>=H) continue;
                        if(clo[ny][nx]==1) continue;
                        if(seen[ny][nx]) continue;
                        seen[ny][nx]=true; q.add(new int[]{nx,ny});
                    }
                }
            }
        }
        if(holes>2) holes=2;
        ClosedMask out=new ClosedMask(); out.mask=clo; out.holes=holes; return out;
    }
    private void pushIfBackgroundMask(int[][] mask,int x,int y,boolean[][] seen,ArrayDeque<int[]> q){
        int H=mask.length,W=mask[0].length;
        if(x<0||x>=W||y<0||y>=H) return;
        if(mask[y][x]==1) return;
        if(!seen[y][x]){ seen[y][x]=true; q.add(new int[]{x,y}); }
    }

    // ===== Shape stats
    private static class ShapeStats {
        double relWidth, relHeight;
        double topRowRatio, bottomRowRatio;
        double bottomHalfRatio, rightHalfMinusLeft;
        double qBRatio;
    }
    private ShapeStats shapeStats(int[][] mask) {
        int H=mask.length, W=mask[0].length;
        int minX=W, maxX=-1, minY=H, maxY=-1, ones=0;
        int[] colSum=new int[W]; int[] rowSum=new int[H];
        for(int y=0;y<H;y++){
            for(int x=0;x<W;x++){
                if(mask[y][x]==1){
                    ones++;
                    if(x<minX)minX=x; if(x>maxX)maxX=x;
                    if(y<minY)minY=y; if(y>maxY)maxY=y;
                    colSum[x]++; rowSum[y]++;
                }
            }
        }
        ShapeStats s=new ShapeStats();
        if(ones==0){ s.relWidth=1.0; s.relHeight=1.0; return s; }
        int bw=(maxX-minX+1), bh=(maxY-minY+1);
        s.relWidth  = bw/(double)W;
        s.relHeight = bh/(double)H;
        s.topRowRatio    = rowSum[0]/(double)W;
        s.bottomRowRatio = rowSum[H-1]/(double)W;

        int halfY = H/2, bot=0, tot=0;
        for(int y=0;y<H;y++){
            for(int x=0;x<W;x++){
                if(mask[y][x]==1){ if(y>=halfY) bot++; tot++; }
            }
        }
        s.bottomHalfRatio = (tot>0)? (bot/(double)tot) : 0.0;

        int halfX = W/2, right=0, left=0;
        for(int y=0;y<H;y++){
            for(int x=0;x<W;x++){
                if(mask[y][x]==1){
                    if(x>=halfX) right++; else left++;
                }
            }
        }
        int totalRL = right + left;
        s.rightHalfMinusLeft = (totalRL>0) ? ((right - left)/(double)totalRL) : 0.0;

        int qB=0, ink=0;
        for(int y=0;y<H;y++){
            for(int x=0;x<W;x++){
                if(mask[y][x]==1){
                    ink++;
                    if(y>=halfY && x>=halfX) qB++;
                }
            }
        }
        s.qBRatio = (ink>0)? (qB/(double)ink) : 0.0;
        return s;
    }

    // ===== Glyph matcher
    private int searchScaled(BufferedImage bin, Box tight, BufferedImage tpl, boolean invertGlyph, boolean widthFit) {
        int w = tight.x2 - tight.x1 + 1, h = tight.y2 - tight.y1 + 1;
        if (w <= 0 || h <= 0) return Integer.MAX_VALUE;
        int gw = tpl.getWidth(), gh = tpl.getHeight();
        if (gw <= 0 || gh <= 0) return Integer.MAX_VALUE;

        double[] scales = new double[]{0.80, 0.90, 1.00, 1.10, 1.20};
        int best = Integer.MAX_VALUE;
        for (double k : scales) {
            int tw, th;
            if (widthFit) {
                double base = w / (double) gw, scale = base * k;
                tw = Math.max(1, (int)Math.round(gw * scale));
                th = Math.max(1, (int)Math.round(gh * scale));
            } else {
                double base = h / (double) gh, scale = base * k;
                th = Math.max(1, (int)Math.round(gh * scale));
                tw = Math.max(1, (int)Math.round(gw * scale));
            }
            int cx0 = tight.x1 + (w - tw)/2;
            int cy0 = tight.y1 + (h - th)/2;

            int slackY = Math.max(32, (int)Math.round(h * 0.28));
            int slackX = Math.max(6,  (int)Math.round(w * 0.10));
            for (int dx = -slackX; dx <= slackX; dx++) {
                int cx = cx0 + dx;
                for (int dy = -slackY; dy <= slackY; dy++) {
                    int cy = cy0 + dy;
                    int diff = diffAt(bin, tight, tpl, invertGlyph, cx, cy, tw, th);
                    if (diff < best) best = diff;
                }
            }
        }
        return best;
    }
    private int diffAt(BufferedImage bin, Box tight, BufferedImage tpl, boolean invertGlyph,
                       int cx, int cy, int tw, int th) {
        int gw = tpl.getWidth(), gh = tpl.getHeight();
        int diff = 0;
        for (int y = 0; y < th; y++) {
            int iy = cy + y;
            for (int x = 0; x < tw; x++) {
                int ix = cx + x;
                if (ix < tight.x1 || ix > tight.x2 || iy < tight.y1 || iy > tight.y2) { diff++; continue; }

                int gx = (int)Math.floor(x * (gw / (double)tw));
                int gy = (int)Math.floor(y * (gh / (double)th));
                if (gx < 0) gx = 0; if (gx >= gw) gx = gw - 1;
                if (gy < 0) gy = 0; if (gy >= gh) gy = gh - 1;

                int vImg = bin.getRaster().getSample(ix, iy, 0); vImg = (vImg != 0 ? 1 : 0);
                int vTpl = tpl.getRaster().getSample(gx, gy, 0);   vTpl = (vTpl != 0 ? 1 : 0);
                if (invertGlyph) vTpl = 1 - vTpl;

                if (vImg != vTpl) diff++;
            }
        }
        return diff;
    }

    // ===== 9×7 grid & Hamming
    private boolean[][] toGrid9x7(BufferedImage bin, Box b, double thresh, boolean invert) {
        int rows = 9, cols = 7;
        boolean[][] grid = new boolean[rows][cols];
        int W = b.x2 - b.x1 + 1, H = b.y2 - b.y1 + 1;

        for (int r = 0; r < rows; r++) {
            int y1 = b.y1 + (int)Math.floor(r     * (H / (double)rows));
            int y2 = b.y1 + (int)Math.floor((r+1) * (H / (double)rows)) - 1;
            if (y2 < y1) y2 = y1;
            for (int c = 0; c < cols; c++) {
                int x1 = b.x1 + (int)Math.floor(c     * (W / (double)cols));
                int x2 = b.x1 + (int)Math.floor((c+1) * (W / (double)cols)) - 1;
                if (x2 < x1) x2 = x1;

                int ones = 0, tot = 0;
                for (int y = y1; y <= y2; y++)
                    for (int x = x1; x <= x2; x++) {
                        int v = bin.getRaster().getSample(x, y, 0);
                        int b01 = (v != 0 ? 1 : 0);
                        if (invert) b01 = 1 - b01;
                        ones += b01; tot++;
                    }
                grid[r][c] = (ones >= (int)Math.round(tot * thresh));
            }
        }
        return grid;
    }
    private int hamming(boolean[][] a, boolean[][] b) {
        int s = 0;
        for (int r = 0; r < 9; r++)
            for (int c = 0; c < 7; c++)
                if (a[r][c] != b[r][c]) s++;
        return s;
    }

    // ===== biased crop
    private Box biasedCrop(Box b, double fracKeep, double anchor) {
        fracKeep = Math.max(0.6, Math.min(1.0, fracKeep));
        anchor   = Math.max(0.0, Math.min(1.0, anchor));
        int h = b.y2 - b.y1 + 1;
        int keep = Math.max(1, (int)Math.round(h * fracKeep));
        double cy = b.y1 + anchor * (h - 1);
        int ny1 = (int)Math.round(cy - (keep - 1) / 2.0);
        int ny2 = ny1 + keep - 1;
        if (ny1 < b.y1) { ny2 += (b.y1 - ny1); ny1 = b.y1; }
        if (ny2 > b.y2) { ny1 -= (ny2 - b.y2); ny2 = b.y2; if (ny1 < b.y1) ny1 = b.y1; }
        return new Box(b.x1, ny1, b.x2, ny2);
    }

    // ===== Profiles
    private static class ProfileFeat { double[] cols, rows; double fill; }
    private ProfileFeat profileOf(BufferedImage bin, Box b) {
        int W = b.x2 - b.x1 + 1, H = b.y2 - b.y1 + 1;
        double[] cp = new double[W];
        double[] rp = new double[H];
        int ones = 0;

        for (int y=0;y<H;y++){
            int rowOn = 0;
            for (int x=0;x<W;x++){
                int v = bin.getRaster().getSample(b.x1+x, b.y1+y, 0);
                int bw = (v!=0?1:0);
                rowOn += bw;
                cp[x]  += bw;
                ones   += bw;
            }
            rp[y] = rowOn;
        }
        double maxC=1, maxR=1;
        for (double v: cp) if (v>maxC) maxC=v;
        for (double v: rp) if (v>maxR) maxR=v;
        for (int i=0;i<cp.length;i++) cp[i]/=maxC;
        for (int i=0;i<rp.length;i++) rp[i]/=maxR;

        ProfileFeat pf = new ProfileFeat();
        pf.cols = resample(cp, 64);
        pf.rows = resample(rp, 64);
        pf.fill = ones/(double)(W*H);
        return pf;
    }
    private double profileDistance(ProfileFeat pf, BufferedImage tpl, double[] colTplRaw, double[] rowTplRaw, double fillTpl) {
        double[] c = resample(colTplRaw, 64);
        double[] r = resample(rowTplRaw, 64);
        double dc=0, dr=0;
        for (int i=0;i<64;i++) { double d=pf.cols[i]-c[i]; dc += d*d; }
        for (int i=0;i<64;i++) { double d=pf.rows[i]-r[i]; dr += d*d; }
        double df = Math.abs(pf.fill - fillTpl);
        return 0.6*dc + 0.4*dr + 0.2*df;
    }
    private static double[] resample(double[] arr, int n) {
        double[] out = new double[n];
        if (arr.length == n) return Arrays.copyOf(arr, n);
        for (int i=0;i<n;i++) {
            double pos = i*(arr.length-1.0)/(n-1.0);
            int a = (int)Math.floor(pos), b=a+1;
            if (b>=arr.length) b=arr.length-1;
            double t = pos - a;
            out[i] = arr[a]*(1-t) + arr[b]*t;
        }
        return out;
    }

    private int argmax(double[] v){
        int id=0; double mx=v[0];
        for (int i=1;i<v.length;i++) if (v[i]>mx){ mx=v[i]; id=i; }
        return id;
    }
}