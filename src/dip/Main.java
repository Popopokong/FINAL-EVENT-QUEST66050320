package dip;

import dip.seg.Segmenter;
import dip.recog.Recognizer;
import dip.recog.TemplateProvider;
import dip.util.Box;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.List;

public class Main {
    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            System.err.println("Usage: java dip.Main <image> [--font=micrenc.ttf] [--debug]");
            return;
        }
        String imagePath = args[0];
        String fontPath = "micrenc.ttf";
        boolean debug = false;
        for (String a : args) {
            if (a.startsWith("--font=")) fontPath = a.substring(7);
            if (a.equals("--debug")) debug = true;
        }

        // 1) อ่านภาพและแปลงเป็นไบนารี (0/1) ด้วย Otsu
        BufferedImage src = ImageIO.read(new File(imagePath));
        BufferedImage bin = toBinaryOtsu(src);

        // 2) เตรียมเทมเพลตจากฟอนต์ (ไบนารี 0/1 เช่นกัน)
        TemplateProvider provider = new TemplateProvider(fontPath);

        // 3) ตัวจำแนก + กล่องแบบ fixed boxes (เหมือนของเพื่อน)
        Recognizer recog = new Recognizer(debug, provider);
        List<Box> boxes = Segmenter.segmentLeftToRight(bin, 6, debug);

        System.out.println("=== Per-digit result (index, box, tightBox, center) ===");
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < boxes.size(); i++) {
            Box b = boxes.get(i);
            Recognizer.Result r = recog.recognizeWithScore(bin, b);
            sb.append(r.digit);
            System.out.printf("pos %d: box %s  tight=%s  center=(%d,%d)  -> %d  (score=%d)%n",
                    i + 1, b, r.tightBox, (b.x1 + b.x2) / 2, (b.y1 + b.y2) / 2, r.digit, r.score);
        }
        System.out.println("Decoded: " + sb.toString());
    }

    
    private static BufferedImage toBinaryOtsu(BufferedImage img) {
        int w = img.getWidth(), h = img.getHeight();

        // to grayscale
        BufferedImage gray = new BufferedImage(w, h, BufferedImage.TYPE_BYTE_GRAY);
        gray.getGraphics().drawImage(img, 0, 0, null);

        // histogram
        int[] hist = new int[256];
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                hist[gray.getRaster().getSample(x, y, 0)]++;
            }
        }

        // Otsu threshold
        int total = w * h;
        long sum = 0;
        for (int i = 0; i < 256; i++) sum += (long) i * hist[i];
        long sumB = 0;
        int wB = 0;
        double maxVar = -1;
        int thr = 0;
        for (int t = 0; t < 256; t++) {
            wB += hist[t];
            if (wB == 0) continue;
            int wF = total - wB;
            if (wF == 0) break;
            sumB += (long) t * hist[t];
            double mB = sumB / (double) wB, mF = (sum - sumB) / (double) wF;
            double var = wB * (double) wF * (mB - mF) * (mB - mF);
            if (var > maxVar) { maxVar = var; thr = t; }
        }

        BufferedImage bin = new BufferedImage(w, h, BufferedImage.TYPE_BYTE_BINARY);
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                int v = gray.getRaster().getSample(x, y, 0);
                bin.getRaster().setSample(x, y, 0, v < thr ? 1 : 0); // 1=หมึก, 0=พื้น
            }
        }
        return bin;
    }
}
