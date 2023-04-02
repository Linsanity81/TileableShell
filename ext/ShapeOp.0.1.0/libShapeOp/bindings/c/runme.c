#include "API.h"
#include <stdio.h>

//print array
void printPoints(ShapeOpScalar *a, int n) {
  int r, c;
  for (c = 0; c < n; c++) {
	printf("Point %i : ( ", c);

	ShapeOpScalar *current_pt = a + c * 3;
    for (r = 0; r < 3; r++){
      printf("%e ", current_pt[r]);
    }

    printf(" )\n");
  }
}

int main() {
  ShapeOpScalar p[12]; //column major
  p[0] = 0.; p[3] = 0.5; p[6] = 0.5; p[9] = 0.;
  p[1] = 0.; p[4] = 0.; p[7] = 1.; p[10] = 1.;
  p[2] = 0.; p[5] = 1.; p[8] = 0.; p[11] = 1.;
  printf("Input points:\n");
  printPoints(p, 4);
  struct ShapeOpSolver *s = shapeop_create();
  shapeop_setPoints(s, p, 4);
  ShapeOpScalar weight = 1.0;
  //add a plane constraint to all the vertices.
  {
    int ids[4];
    ids[0] = 0; ids[1] = 1; ids[2] = 2; ids[3] = 3;
    if( 0 > shapeop_addConstraint(s, "Plane", ids, 4, weight) ){
    	printf("shapeop_addConstraint failed. Exiting.\n"); return -1;
    }
  }

  //add a closeness constraint to the 1st vertex.
  {
	int ids[1]; ids[0] = 0;
    if( 0 > shapeop_addConstraint(s, "Closeness", ids, 1, weight) ){
    	printf("shapeop_addConstraint failed. Exiting.\n"); return -1;
    }
  }
  //add a closeness constraint to the 4th vertex.
  {
	int ids[1]; ids[0] = 3;
    if( 0 > shapeop_addConstraint(s, "Closeness", ids, 1, weight) ){
    	printf("shapeop_addConstraint failed. Exiting.\n"); return -1;
    }
  }
  //add a rigid constraint between 1st and 4th vertex.
  {
    int ids[2]; ids[0] = 0; ids[1] = 3;
    int cid = shapeop_addConstraint(s, "Rigid", ids, 2, weight);
    if (0 > cid){
      printf("shapeop_addConstraint failed. Exiting.\n"); return -1;
    }
    ShapeOpScalar scalars[12];
    scalars[0] = 0.; scalars[1] = 0.; scalars[2] = 0.;
    scalars[3] = 0.;scalars[4] = 0.; scalars[5] = 0.;
    scalars[6] = 0.; scalars[7] = 0.; scalars[8] = 0.;
    scalars[9] = 0.;scalars[10]= 0.; scalars[11]= 0.;
    int err_code = shapeop_editConstraint(s, "Rigid", cid, scalars, 12);
    if ( err_code != SO_SUCCESS ){
      printf("shapeop_editConstraint failed with error code %d. Exiting.\n", err_code); 
      return -1;
    }
  }
  if(shapeop_init(s)){ printf("shapeop_init failed. Exiting.\n"); return -1; }
  if(shapeop_solve(s, 10)){ printf("shapeop_solve failed. Exiting.\n"); return -1; }
  shapeop_getPoints(s, p, 4);
  shapeop_delete(s);
  printf("Output points:\n");
  printPoints(p, 4);
  return 0;
}
