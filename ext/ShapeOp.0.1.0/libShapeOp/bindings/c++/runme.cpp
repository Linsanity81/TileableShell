#include "Solver.h"
#include "Constraint.h"
#include <iostream>

void print_points(const ShapeOp::Matrix3X &p)
{
	for(int i = 0; i < 6; ++ i){
		std::cout << "v ";
		ShapeOp::Vector3 current_pt = p.col(i);
		std::cout << current_pt.transpose();
		std::cout << std::endl;
//		std::cout << "f " << std::endl;
	}
    std::cout << "f 1 2 3 4" << std::endl;
    std::cout << "f 3 4 5 6" << std::endl;
}

int main() {
    ShapeOp::Matrix3X p; //column major
    p.resize(3, 6);

    // The << operator reads elements row by row
    p << 0.0, 1.0, 1.0, 0.0, 0.0, 1.0,
         -1.0, -1.0, 0.0, 0.0, 1.0, 1.0,
         0.0, 0.0, 1.0, 1.0, 0.0, 0.0;

    std::cout <<  "Input points:" << std::endl;
    print_points(p);

    ShapeOp::Solver s;
    s.setPoints(p);
    ShapeOp::Scalar weight = 1.0;

    //add a plane constraint
    {
        std::vector<int> id_vector;
        id_vector.push_back(0); id_vector.push_back(1);
        id_vector.push_back(2); id_vector.push_back(3);
        auto c = std::make_shared<ShapeOp::PlaneConstraint>(id_vector, weight, s.getPoints());
        s.addConstraint(c);
    }

    //add a plane constraint
    {
        std::vector<int> id_vector;
        id_vector.push_back(2); id_vector.push_back(3);
        id_vector.push_back(4); id_vector.push_back(5);
        auto c = std::make_shared<ShapeOp::PlaneConstraint>(id_vector, weight, s.getPoints());
        s.addConstraint(c);
    }

    //add a bending constraint
    {
        std::vector<int> id_vector;
        id_vector.push_back(2); id_vector.push_back(3);
        id_vector.push_back(0); id_vector.push_back(5);
        auto c = std::make_shared<ShapeOp::BendingConstraint>(id_vector, weight, s.getPoints(), 0.5, 0.5);
        s.addConstraint(c);
    }

    //add a bending constraint
    {
        std::vector<int> id_vector;
        id_vector.push_back(2); id_vector.push_back(3);
        id_vector.push_back(1); id_vector.push_back(4);
        auto c = std::make_shared<ShapeOp::BendingConstraint>(id_vector, weight, s.getPoints(), 0.5, 0.5);
        s.addConstraint(c);
    }


    // add a edge constraint
    {
        std::vector<int> id_vector;
        id_vector.push_back(2); id_vector.push_back(3);
        auto c = std::make_shared<ShapeOp::EdgeStrainConstraint>(id_vector, weight, s.getPoints(), 2, 2);
        s.addConstraint(c);
    }

    //add a closeness constraint to the 4th vertex.
    {
//        std::vector<int> id_vector;
//        id_vector.push_back(0); id_vector.push_back(1);
//        id_vector.push_back(2); id_vector.push_back(3);
//        auto c = std::make_shared<ShapeOp::ClosenessConstraint>(id_vector, weight, s.getPoints());
//        s.addConstraint(c);
    }

    //add a rigid constraint between 1st and 4th vertex.
    {
//        std::vector<int> id_vector;
//        id_vector.push_back(0); id_vector.push_back(1);
//        id_vector.push_back(2); id_vector.push_back(3);
//
//        ShapeOp::Matrix34 s1;
//        s1 << 0.0, 0.5, 0.5, 0.0,
//                0.0, 0.0, 1.0, 1.0,
//                0.1, 1.0, 0.0, 1.0;
//
//        auto c = std::make_shared<ShapeOp::SimilarityConstraint>(id_vector, weight, s1, false);
//
//        s.addConstraint(c);
    }
    s.initialize();
    s.solve(10);
    p = s.getPoints();
    std::cout << "Output points:" << std::endl;
    print_points(p);

    return 0;
}
