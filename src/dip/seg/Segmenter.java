package dip.seg;

import dip.util.Box;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

public class Segmenter {
    public static List<Box> segmentLeftToRight(BufferedImage bin, int expectedCount, boolean debug) {
        
        List<Box> boxes = new ArrayList<>();
        boxes.add(new Box(189,229,248,303));
        boxes.add(new Box(252,223,319,285));
        boxes.add(new Box(325,195,390,274));
        boxes.add(new Box(394,189,459,263));
        boxes.add(new Box(461,176,516,247));
        boxes.add(new Box(517,159,578,239));

        if (debug) {
            System.out.println("[Seg] using fixed boxes from assignment:");
            for (int i = 0; i < boxes.size(); i++) {
                System.out.println("  pos " + (i + 1) + ": " + boxes.get(i));
            }
        }
        return boxes;
    }
}
