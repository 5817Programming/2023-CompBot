public class Grid{
    boolean[][] grid;
    public Grid(int xSize, int ySize){
        grid = new boolean[xSize][ySize];
        java.util.Arrays.fill(grid[0], false);
        java.util.Arrays.fill(grid[1], false);
    }

    public void fill(int x, int y){
        grid[x][y] = true;
    }
    public void clear(int x, int y){
        grid[x][y] = false;
    }

    public void clear(){
        java.util.Arrays.fill(grid[0], false);
        java.util.Arrays.fill(grid[1], false);
    }
    public double evaluateNode(int x, int y,double distanceWeight,double neighborWeight,double distanceWeight, double middleWeight, double scoreWeight){
        boolean neighbors;
        double score = 0;
        if(x >= 4 && x <= 6){
            score+middleWeight;
        }
        if(x == 2){
            score += 5*scoreWeight;
        }else if(x == 1){
            score += 3*scoreWeight;
        }else if(x == 0){
            score += 2* scoreWeight;
        }

        if(grid[x-1] == true && x[x+1] == true){
            score += 5 * scoreWeight;
        }
        else if(grid[x-1] == true && grid[x-2] == true){
            score += 5 * scoreWeight;
        }
        else if(grid[x+1] == true && grid[x+2] == true){
            score += 5 * scoreWeight;
        }
        else if(grid[x-1] == true){
            neighbors = true;
        }
        else if(grid[x-2] == true){
            neighbors = true;
        }
        else if(grid[x+1] == true){
            neighbors = true;
        }
        else if(grid[x+2] == true){
            neighbors = true;
        }
        if(neighbors){
            score += neighborWeight;
        }

        score/((distance)); 



    }
}