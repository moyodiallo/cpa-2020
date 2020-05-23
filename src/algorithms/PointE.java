package algorithms;

import java.awt.Point;

class PointE extends Point {
    /**
     *
     */
    private static final long serialVersionUID = 1L;
    
    public int etiquette;
    public boolean feuille;

    PointE(Point p, int e) {
        super(p);
        etiquette = e;
        feuille = false;
    }

    PointE(Point p) {
        super(p);
        etiquette = -1;
        feuille = false;
    }
}