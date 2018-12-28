#include "../src/tools.cpp"
#include <gtest/gtest.h>
 
Tools tools;

TEST(CalculateRMSETest, CorrectCalculation) { 
  vector<VectorXd> estimations;  
  VectorXd estimate(4);
  estimate << 1.1, 2.2, 3.3, 4.4;
  estimations.push_back(estimate);
  estimate << 5.5, 6.6, 7.7, 8.8;
  estimations.push_back(estimate);
  
  vector<VectorXd> ground_truth;
  VectorXd gt(4);
  gt << 1.0, 2.0, 3.0, 4.0;
  ground_truth.push_back(gt);
  gt << 5.0, 6.0, 7.0, 8.0;
  ground_truth.push_back(gt);
   
  VectorXd rmseExpected(4);
  rmseExpected << 0.36055512754, 0.4472135955, 0.53851648071, 0.63245553203;

  VectorXd rmseActual = tools.CalculateRMSE(estimations, ground_truth);
  
  for (unsigned int i=0; i < estimations.size(); i++) {
    ASSERT_FLOAT_EQ(rmseExpected[i], rmseActual[i]);
  }
}

TEST(CalculateRMSETest, UnequalLengths) {
  vector<VectorXd> estimations;  
  VectorXd estimate(4);
  estimate << 1.1, 2.2, 3.3, 4.4;
  estimations.push_back(estimate);
  estimate << 5.5, 6.6, 7.7, 8.8;
  estimations.push_back(estimate);
  
  vector<VectorXd> ground_truth;
  VectorXd gt(4);
  gt << 1.0, 2.0, 3.0, 4.0;
  ground_truth.push_back(gt);

  VectorXd rmseEmpected(4);
  rmseEmpected << 0,0,0,0;
  
  VectorXd rmseActual = tools.CalculateRMSE(estimations, ground_truth);

  ASSERT_EQ(rmseEmpected, rmseActual);
}

TEST(CalculateRMSETest, OneHasZeroElements) {
  vector<VectorXd> estimations;  
  
  vector<VectorXd> ground_truth;
  VectorXd gt(4);
  gt << 1.0, 2.0, 3.0, 4.0;
  ground_truth.push_back(gt);

  VectorXd rmseEmpected(4);
  rmseEmpected << 0,0,0,0;
  
  VectorXd rmseActual = tools.CalculateRMSE(estimations, ground_truth);

  ASSERT_EQ(rmseEmpected, rmseActual);
}
 
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}