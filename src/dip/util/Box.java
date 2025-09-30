package dip.util;

public class Box {
    public int x1, y1, x2, y2;
    public Box(int x1, int y1, int x2, int y2) { this.x1 = x1; this.y1 = y1; this.x2 = x2; this.y2 = y2; }
    public int width()  { return x2 - x1 + 1; }
    public int height() { return y2 - y1 + 1; }
    @Override public String toString() { return String.format("[%d,%d]-[%d,%d]", x1, y1, x2, y2); }
}
