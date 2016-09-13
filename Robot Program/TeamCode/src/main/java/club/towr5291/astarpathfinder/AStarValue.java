package club.towr5291.astarpathfinder;

/**
 * Created by ianhaden on 5/09/16.
 */
public class AStarValue {

     // property help us to keep data
    public int ID;
    public double FValue;
    public double GValue;
    public double HValue;
    public int Parent;
    public int xvalue;
    public int yvalue;

    public AStarValue (int ID, double FValue, double GValue, double HValue, int Parent, int xvalue, int yvalue) {
        this.ID = ID;
        this.FValue = FValue;
        this.GValue = GValue;
        this.HValue = HValue;
        this.Parent = Parent;
        this.xvalue = xvalue;
        this.yvalue = yvalue;
    }
    public AStarValue() {
        this(0, 0, 0, 0, 0, 0, 0);
    }
}
