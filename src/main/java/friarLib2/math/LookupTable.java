package friarLib2.math;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Comparator;

public class LookupTable 
{
    public static LookupTable CreateNormalized()
    {
        LookupTable table = new LookupTable();
        table.AddValue(0, 0);
        table.AddValue(1, 1);
        
        return table;
    }
    
    private ArrayList<Point2D.Double> Nodes = new ArrayList<Point2D.Double>();
    
    public LookupTable AddValue(double input, double output)
    {
        Nodes.add(new Point2D.Double(input, output));
        Nodes.sort(Comparator.comparingDouble(a -> a.x));
        
        return this;
    }
    
    public double GetValue(double input)
    {
        assert Nodes.size() > 1 : "LookupTable requires at least two values to lerp between";

        var numNodes = Nodes.size();
        
        // -- Clamp Low
        var low = Nodes.get(0).x;
        if (input <= low) 
        {
            return low; 
        }
        
        // -- Clamp High
        var high = Nodes.get(numNodes - 1).x;
        if (input >= high)
        {
            return high; 
        }
        
        Point2D.Double Left = Nodes.get(0);
        Point2D.Double Right = null;
        
        // -- Find the node we're past
        for (int i = 0; i < numNodes; i++) 
        {
            var Node = Nodes.get(i);
            if (input > Node.x) 
            {
                Left = Node;
            }
            else
            {
                Right = Nodes.get(i);
                break;
            }
        }

        assert Right != null : "Didn't find a Right value";
        return FriarMath.Remap(input, Left.x, Right.x, Left.y, Right.y);
    }
}
