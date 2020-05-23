package algorithms;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;

public class DefaultTeam {

    public Tree2D calculSteiner(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints) {

        // Les hitPoints traduit en pointE(x,y,etiquette)
        // On transforme en PointE a cause de l'etiquette 
        // car cela permet d'optimiser kruskal
        // c-a-d eviter de repeter certaines des boucles
        ArrayList<PointE> listPoint = new ArrayList<PointE>();
        int etiq = 1;
        for (Point p : hitPoints) {
            listPoint.add(new PointE(p, etiq++));
        }

        // kruskal
        ArrayList<Edge> edges = kruskal(listPoint);

        // all to all paths application
        int[][] paths = calculShortestPaths(points, (double)edgeThreshold);

        ArrayList<Edge> eSolution = new ArrayList<Edge>();

        // trouver les liens entre 2 points de la solution 
        // donnee par kruskal
        for (Edge e : edges) {

            int indexP = points.indexOf(e.p);
            int indexQ = points.indexOf(e.q);
            int indexK;

            do {
                indexK = paths[indexP][indexQ];
                eSolution.add(new Edge(new PointE(points.get(indexP)), new PointE(points.get(indexK))));
                indexP = indexK;
            } while (indexP != indexQ);
        }

        System.out.println("eSolution "+eSolution.size());

        return constructTree(eSolution);
    }

    @SuppressWarnings("unchecked")
    public Tree2D calculSteinerBudget(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints) throws Exception {

        // Les hitPoints traduit en pointE(x,y,etiquette)
        // On transforme en PointE a cause de l'etiquette 
        // car cela permet d'optimiser kruskal
        // c-a-d eviter de repeter certaines des boucles
        ArrayList<PointE> listHitPointE = new ArrayList<PointE>();
        ArrayList<PointE> listPointE    = new ArrayList<PointE>();

        
        for (Point point : points) {
            PointE pointE = new PointE(point);//Pour avoir les memes references avec "listHitPointE" et "listPointE"
            listPointE.add(pointE);
            if(hitPoints.contains(point)){
                listHitPointE.add(pointE);
            }
        }

        //Donner des etiquettes differentes
        int etiq = 1;
        for (PointE pointE : listHitPointE) {
            pointE.etiquette = etiq++;
        }

        // kruskal
        ArrayList<Edge> edges = kruskal(listHitPointE);

        // All to All path, pour le calcul de la distance minimale
        int[][] paths = calculShortestPaths(points, (double)edgeThreshold);

        //Stocker les solutions comme des edges dans un premier temps
        ArrayList<Edge> eSolution  = new ArrayList<Edge>();

        double distanceTotal = 0; // distance Total de l'Abre Trouver
        int indexP;
        int indexQ;
        int indexK;
        double distance;
        PointE pointP;
        PointE pointK;

        // trouver les liens entre 2 points de la solution 
        // donnee par kruskal
        for (Edge e : edges) {

            indexP = listPointE.indexOf(e.p);
            indexQ = listPointE.indexOf(e.q);

            do {
                indexK = paths[indexP][indexQ];
                pointP = listPointE.get(indexP);
                pointK = listPointE.get(indexK);
                distance = pointK.distance(pointP);
                distanceTotal = distanceTotal + distance;
                Edge edge = new Edge(pointP, pointK, distance); //les Edges avec leur distance(utile pour l'optimisation)
                eSolution.add(edge);
                indexP = indexK;
            } while (indexP != indexQ);
        }

        // Les Edges contenant une feuille
        // Pour savior quoi supprimer a chaque fois
        // afin de gerer le budget
        ArrayList<Edge> edgeExtrem = new ArrayList<Edge>();
        for (Edge edge : eSolution) {
            if(edge.p.feuille || edge.q.feuille)
            {
                edgeExtrem.add(edge);
            }
        }

        System.out.println("hitpoints     "+hitPoints.size());
        System.out.println("distanceTotal "+distanceTotal);
        System.out.println("edgeExtrem    "+edgeExtrem.size());

        Point pointMere = hitPoints.get(0);

        // restreindre sur le budget 1664
        double budget = 1664;
        PointE point  = null;
        int neighbour;
        while (distanceTotal > budget) {

            // Extraire un edge dans les edges contenant les feuilles
            Edge e = Collections.max(edgeExtrem);
            edgeExtrem.remove(e);

            // eviter de supprimer le point Mere, meme ce dernier
            // au cas ou il se trouve sur une feuile
            if( (e.q.feuille && e.q.equals(pointMere))  || (e.p.feuille && e.p.equals(pointMere)))
            {
                continue;
            }

            eSolution.remove(e);

            //La distance est retrancher apres avoir supprimer
            //le Edge de la solution(Tres essentiel)
            distanceTotal = distanceTotal - e.distance;

            //Trouver le point a cote de la feuille
            //Pour pouvoir 
            if (e.p.feuille)
                point = e.q; // le point a cote de la feuille
            else if(e.q.feuille)
                point = e.p; // le point a cote de la feuille
            else{
                System.err.println("echec");
                System.exit(-1);
            } 

            neighbour = 0;
            Edge ed = null;
            // possibilite que "point" soif une feuille
            for (Edge v : eSolution) {
                if ( (v.q.equals(point) || v.p.equals(point))) {
                    neighbour++;
                    ed = v;
                }
                
                if(neighbour > 1) {
                    // "point" n'est pas une feuille
                    // puisque le nombre de ses voisins sont superieur a 1 
                    break;
                }
            }

            if (neighbour == 1) {
                point.feuille = true;
                edgeExtrem.add(ed);
            }
        }

        Tree2D tree = constructTree(eSolution);

        /*
        LinkedList<Tree2D> workinglist = new LinkedList<>();
        workinglist.push(tree);

        if(tree.getRoot().equals(pointMere)){
            System.out.println("fond in Tree");
        }

        while(!workinglist.isEmpty()){
            Tree2D p = workinglist.poll();

            for(Tree2D t :p.getSubTrees()){
                if(!t.getRoot().equals(pointMere)){
                    workinglist.add(t);
                }else{
                    System.out.println("Found in Tree");
                    workinglist.add(t);
                }
            }
        }
        */
        
        return tree;
    }


    @SuppressWarnings("unchecked")
    public ArrayList<Edge> kruskal(ArrayList<PointE> points) {

        //Construction des Edges
        LinkedList<Edge> edges = new LinkedList<Edge>();
        for (int i = 0; i < points.size(); i++) {
            for (int j = 0; j < points.size(); j++) {
                if(i!=j) {
                    PointE p1 = points.get(i);
                    PointE p2 = points.get(j);
                    edges.add(new Edge(p1,p2,p1.distance(p2)));
                }
            }
        }

        // Trie des edges en fonction des distances
        Collections.sort(edges);
        ArrayList<Edge> eSolution = new ArrayList<Edge>();

        //On evitera de choisir un Edge dont les 2 points 
        //sont deja dans l'abre couvrant(c'est-a-dire leur etiquette est negatif)
        while (!edges.isEmpty()) {
            Edge e = edges.poll();
            
            //On ajoute le Edge si c'est possible
            //c'est-a-dire que les 2 noeuds, il ya au moins 1 
            //qui n'appartient pas un Edge deja choisi
            //Un noeud ayant une etiquette negatif appartient deja 
            //a l'abre en cours de construction


            if(e.q.etiquette < 0 && e.p.etiquette < 0 && e.q.etiquette != e.p.etiquette) {
                int etiquetteTo   = e.p.etiquette;
                int etiquetteFrom = e.q.etiquette;

                // Une contamination d'etiquette
                // C'est-a-dire que si 2 sous-arbre ne sont pas connecter
                // On les connectent(choisi) en un sous arbre pour former un
                // nouveau arbre avec une etiquette  negatif pour tout l'arbre
                for (Edge edge : edges) {
                    if(edge.q.etiquette == etiquetteFrom){
                        edge.q.etiquette = etiquetteTo;
                    }

                    if(edge.p.etiquette == etiquetteFrom){
                        edge.p.etiquette = etiquetteTo;
                    }
                }

                //Pas de feuille puisqu'il appartient deja 
                //a 2 sous-arbre different
                e.p.feuille = false;
                e.q.feuille = false;
                eSolution.add(e);

            }else if (e.p.etiquette > 0 ||  e.q.etiquette > 0) {
                // 2 point qui n'ont pas ete connecter pour le moment
                // l'un a une etiquette positif

                //la feuille est un point isolé
                if(e.p.etiquette > 0){
                    e.p.feuille = true;
                }else{
                    e.p.feuille = false;
                }

                //La feuille est un point isolé
                if(e.q.etiquette > 0){
                    e.q.feuille = true;
                }else {
                    e.q.feuille = false;
                }

                //On fait un choisie d'etiquette commun
                //Priorité à l'etiquette negatif, puisqu'il appartient deja à un arbre
                int etiquetteCommun = e.p.etiquette < 0 ? e.p.etiquette : e.q.etiquette;

                if(etiquetteCommun < 0){
                    e.p.etiquette = etiquetteCommun;
                    e.q.etiquette = etiquetteCommun;
                }else {
                    e.p.etiquette = -etiquetteCommun;
                    e.q.etiquette = -etiquetteCommun;
                }

                eSolution.add(e);
            }
        }
        return eSolution;
    }

    @SuppressWarnings("unchecked")
    public Tree2D constructTree(ArrayList<Edge> edges) {

        //Consturction de Tree2D a partir des Edges
        
        // Construction de l'arbre
        Tree2D root = new Tree2D(edges.get(0).p, new ArrayList<Tree2D>());


        LinkedList<Edge> linkedListEgdes = new LinkedList<Edge>();
        for (Edge edge : edges) {
            linkedListEgdes.add(edge);
        }

        LinkedList<Tree2D> lifo = new LinkedList<Tree2D>();
        lifo.add(root);
        
        while(!lifo.isEmpty()) {
            Iterator it = linkedListEgdes.iterator();
            Tree2D tree = lifo.poll();

            while(it.hasNext()) {
                Edge e = (Edge)it.next();
                if (e.p.equals(tree.getRoot())) {
                    Tree2D r = new Tree2D(e.q, new ArrayList<Tree2D>());
                    tree.getSubTrees().add(r);
                    lifo.add(r);
                    it.remove();
                } else if (e.q.equals(tree.getRoot())) {
                    Tree2D r = new Tree2D(e.p, new ArrayList<Tree2D>());
                    tree.getSubTrees().add(r);
                    lifo.add(r);
                    it.remove();
                }
            }
        }

        return root;
    }

    public int[][] calculShortestPaths(ArrayList<Point> points, double edgeThreshold) {
        int[][] paths = new int[points.size()][points.size()];
        for (int i = 0; i < paths.length; i++)
            for (int j = 0; j < paths.length; j++)
                paths[i][j] = i;

        // stocker les distances minimales temporaires
        double[][] distances = new double[points.size()][points.size()];

        for (int i = 0; i < points.size(); i++) {
            for (int j = 0; j < points.size(); j++) {

                double distance = points.get(i).distance(points.get(j));
                if (distance < edgeThreshold) {
                    paths[i][j] = j;
                    distances[i][j] = distance;
                } else
                    distances[i][j] = Double.MAX_VALUE;
            }
        }

        for (int k = 0; k < points.size(); k++) {
            for (int i = 0; i < points.size(); i++) {
                for (int j = 0; j < points.size(); j++) {
                    double distance = distances[i][k] + distances[k][j];
                    if (distances[i][j] > distance) {
                        distances[i][j] = distance;
                        paths[i][j] = paths[i][k];
                    }
                }
            }

        }
        return paths;
    }

}