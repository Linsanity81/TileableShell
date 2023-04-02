#!/usr/bin/python
#/usr/local/bin/python3
import shapeopPython

#print array
def printArray(a, n):
  for c in range(0,3):
    for r in range(0,n):
      print a[r * 3 + c],
    print ""

def main():
  #create points with the swig doubleArray proxy
  #http://www.swig.org/Doc3.0/SWIGDocumentation.html#Library_carrays
  p = shapeopPython.doubleArray(12) #column major
  p[0] = 0.; p[3] = 0.5; p[6] = 0.5; p[9] = 0.;
  p[1] = 0.; p[4] = 0.; p[7] = 1.; p[10] = 1.;
  p[2] = 0.; p[5] = 1.; p[8] = 0.; p[11] = 1.;
  print "Input points:"
  printArray(p, 4)
  s=shapeopPython.shapeop_create()
  shapeopPython.shapeop_setPoints(s, p, 4)
  weight = 1.
  #add a plane constraint to all the vertices.
  ids = shapeopPython.intArray(4)
  ids[0] = 0; ids[1] = 1; ids[2] = 2; ids[3] = 3;
  shapeopPython.shapeop_addConstraint(s, "Plane", ids, 4, weight)
  #add a closeness constraint to the 1st vertex.
  ids = shapeopPython.intArray(1)
  ids[0] = 0;
  shapeopPython.shapeop_addConstraint(s, "Closeness", ids, 1, weight)
  #add a closeness constraint to the 4th vertex.
  ids = shapeopPython.intArray(1)
  ids[0] = 3;
  shapeopPython.shapeop_addConstraint(s, "Closeness", ids, 1, weight)
  shapeopPython.shapeop_init(s)
  shapeopPython.shapeop_solve(s, 10)
  shapeopPython.shapeop_getPoints(s, p, 4)
  shapeopPython.shapeop_delete(s)
  print "Output points:"
  printArray(p, 4)

if __name__ == "__main__":
  main()
