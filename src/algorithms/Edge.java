package algorithms;


import java.lang.Comparable;

public class Edge implements Comparable {
    public PointE p, q;
    double distance;

    Edge(PointE p, PointE q, double distance) {
        this.p = p;
        this.q = q;
        this.distance = distance;
    }

    Edge(PointE p, PointE q) {
        this.p = p;
        this.q = q;
        this.distance = -1;
    }

    @Override
    public int compareTo(Object arg0) {

        Edge e = (Edge) arg0;
        if (e.distance < this.distance) {
            return 1;
        }else if (e.distance > this.distance)
            return -1;
        else 
            return 0;
    }

    @Override
    public boolean equals(Object obj) {
        Edge e = (Edge) obj;
        if(e.q.equals(this.q) && e.p.equals(this.p)){
            return true;
        }else if(e.q.equals(this.p) && e.p.equals(this.q)){
            return true;
        }else{
            return false;
        }
    }
}