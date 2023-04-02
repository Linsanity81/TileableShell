/** Example usage of the java bindings of ShapeOp */
public class runme {
  static {
    System.loadLibrary("shapeopJava");
  }
  public static void printArray(doubleArray a, int n) {
    for (int c = 0; c < 3; c++) {
      for (int r = 0; r < n; r++)
        System.out.printf("%f ", a.getitem(r*3 + c));
      System.out.printf("\n");
    }
  }
  public static void main(String argv[]) {
    //create points with the swig doubleArray proxy
    //http://www.swig.org/Doc3.0/SWIGDocumentation.html#Library_carrays
    doubleArray p = new doubleArray(12); //column major
    p.setitem(0, 0.); p.setitem(3, 0.5); p.setitem(6, 0.5); p.setitem(9, 0.);
    p.setitem(1, 0.); p.setitem(4, 0.); p.setitem(7, 1.); p.setitem(10, 1.);
    p.setitem(2, 0.); p.setitem(5, 1.); p.setitem(8, 0.); p.setitem(11, 1.);
    System.out.println("Input points:");
    printArray(p, 4);
    SWIGTYPE_p_ShapeOpSolver s = shapeopJava.shapeop_create();
    shapeopJava.shapeop_setPoints(s, p.cast(), 4);
    double weight = 1.;
    //add a plane constraint to all the vertices.
    {
      intArray ids = new intArray(4);
      ids.setitem(0, 0); ids.setitem(1, 1); ids.setitem(2, 2); ids.setitem(3, 3);
      shapeopJava.shapeop_addConstraint(s, "Plane", ids.cast(), 4, weight);
    }
    //add a closeness constraint to the 1st vertex.
    {
      intArray ids = new intArray(1);
      ids.setitem(0, 0);
      shapeopJava.shapeop_addConstraint(s, "Closeness", ids.cast(), 1, weight);
    }
    //add a closeness constraint to the 4th vertex.
    {
      intArray ids = new intArray(1);
      ids.setitem(0, 3);
      shapeopJava.shapeop_addConstraint(s, "Closeness", ids.cast(), 1, weight);
    }
    shapeopJava.shapeop_init(s);
    shapeopJava.shapeop_solve(s, 10);
    shapeopJava.shapeop_getPoints(s, p.cast(), 4);
    shapeopJava.shapeop_delete(s);
    System.out.println("Output points:");
    printArray(p, 4);
  }
}
