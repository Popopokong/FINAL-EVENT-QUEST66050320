package dip.recog;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.FileInputStream;

public class TemplateProvider {
    private final BufferedImage[] glyphBin = new BufferedImage[10];
    private final boolean[][][] templates9x7 = new boolean[10][9][7];
    private final double[][] colProfiles = new double[10][]; // normalized per column
    private final double[][] rowProfiles = new double[10][]; // normalized per row
    private final double[] fillRatio = new double[10];       // foreground ratio per digit

    public TemplateProvider(String fontPath) throws Exception {
        Font font = loadFont(fontPath);
        for (int d = 0; d <= 9; d++) {
            BufferedImage g = renderGlyphBinary(font, (char) ('0' + d)); // binary 0/1
            glyphBin[d] = g;
            templates9x7[d] = toGrid9x7(g);
            Profile p = profileOf(g);
            colProfiles[d] = p.cols;
            rowProfiles[d] = p.rows;
            fillRatio[d]   = p.fill;
        }
    }

    public BufferedImage[] getGlyphBin()     { return glyphBin;      }
    public boolean[][][] getTemplates9x7()   { return templates9x7;  }
    public double[][] getColProfiles()       { return colProfiles;   }
    public double[][] getRowProfiles()       { return rowProfiles;   }
    public double[]  getFillRatio()          { return fillRatio;     }

    // ===== font → binary template =====
    private Font loadFont(String path) throws Exception {
        try (FileInputStream fis = new FileInputStream(path)) {
            Font base = Font.createFont(Font.TRUETYPE_FONT, fis);
            return base.deriveFont(Font.PLAIN, 240f);
        }
    }

    private BufferedImage renderGlyphBinary(Font font, char ch) {
        int W = 260, H = 220;
        BufferedImage gray = new BufferedImage(W, H, BufferedImage.TYPE_BYTE_GRAY);
        Graphics2D g = gray.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_OFF);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_OFF);
        g.setColor(Color.WHITE);
        g.fillRect(0, 0, W, H);
        g.setColor(Color.BLACK);
        g.setFont(font);
        FontMetrics fm = g.getFontMetrics();
        int tw = fm.charWidth(ch);
        int th = fm.getAscent();
        int x  = (W - tw) / 2;
        int y  = (H + th) / 2 - fm.getDescent();

        // หนาแนวตั้ง 2 เส้น พอด แยก 1/8 และ 3/7
        g.drawString(String.valueOf(ch), x, y);
        g.drawString(String.valueOf(ch), x, y + 1);
        g.dispose();

        return toBinaryOtsu(gray);
    }

    private BufferedImage toBinaryOtsu(BufferedImage grayInput) {
        int w = grayInput.getWidth(), h = grayInput.getHeight();

        BufferedImage gray = new BufferedImage(w, h, BufferedImage.TYPE_BYTE_GRAY);
        gray.getGraphics().drawImage(grayInput, 0, 0, null);

        int[] hist = new int[256];
        for (int y = 0; y < h; y++) for (int x = 0; x < w; x++)
            hist[gray.getRaster().getSample(x, y, 0)]++;

        int total = w * h; long sum = 0;
        for (int i = 0; i < 256; i++) sum += (long) i * hist[i];

        long sumB = 0; int wB = 0; double maxVar = -1; int thr = 0;
        for (int t = 0; t < 256; t++) {
            wB += hist[t]; if (wB == 0) continue;
            int wF = total - wB; if (wF == 0) break;
            sumB += (long) t * hist[t];
            double mB = sumB / (double) wB, mF = (sum - sumB) / (double) wF;
            double var = wB * (double) wF * (mB - mF) * (mB - mF);
            if (var > maxVar) { maxVar = var; thr = t; }
        }

        BufferedImage bin = new BufferedImage(w, h, BufferedImage.TYPE_BYTE_BINARY);
        for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) {
            int v = gray.getRaster().getSample(x, y, 0);
            bin.getRaster().setSample(x, y, 0, v < thr ? 1 : 0);
        }
        return bin;
    }

    private boolean[][] toGrid9x7(BufferedImage bin) {
        int rows = 9, cols = 7;
        boolean[][] grid = new boolean[rows][cols];

        int W = bin.getWidth(), H = bin.getHeight();
        for (int r = 0; r < rows; r++) {
            int y1 = (int)Math.floor(r     * (H / (double)rows));
            int y2 = (int)Math.floor((r+1) * (H / (double)rows)) - 1;
            if (y2 < y1) y2 = y1;
            for (int c = 0; c < cols; c++) {
                int x1 = (int)Math.floor(c     * (W / (double)cols));
                int x2 = (int)Math.floor((c+1) * (W / (double)cols)) - 1;
                if (x2 < x1) x2 = x1;

                int ones = 0, tot = 0;
                for (int y = y1; y <= y2; y++)
                    for (int x = x1; x <= x2; x++) {
                        int v = bin.getRaster().getSample(x, y, 0);
                        if (v != 0) ones++;
                        tot++;
                    }
                grid[r][c] = (ones * 2 >= tot);  
            }
        }
        return grid;
    }

    // normalized 0..1
    private static class Profile { double[] cols, rows; double fill; }
    private Profile profileOf(BufferedImage bin) {
        int W = bin.getWidth(), H = bin.getHeight();
        double[] cp = new double[W];
        double[] rp = new double[H];
        int ones = 0;
        for (int y=0;y<H;y++){
            int rowOn=0;
            for (int x=0;x<W;x++){
                int v = bin.getRaster().getSample(x,y,0);
                int b = (v!=0?1:0);
                rowOn += b;
                cp[x]  += b;
                ones   += b;
            }
            rp[y] = rowOn;
        }
        // normalize to [0,1]
        double maxC=1, maxR=1;
        for (int x=0;x<W;x++) if (cp[x]>maxC) maxC=cp[x];
        for (int y=0;y<H;y++) if (rp[y]>maxR) maxR=rp[y];
        for (int x=0;x<W;x++) cp[x]/=maxC;
        for (int y=0;y<H;y++) rp[y]/=maxR;
        Profile p = new Profile();
        p.cols = cp; p.rows = rp; p.fill = ones/(double)(W*H);
        return p;
    }
}
