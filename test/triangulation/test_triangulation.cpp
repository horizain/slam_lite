#include <gtest/gtest.h>
#include <include/common_include.h>

/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */
inline bool triangulation(const std::vector<SE3> &poses, const std::vector<Vec3> points, Vec3 &pt_world)
{
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i)
    {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
    {
        // std::cout << "The solution using the SVD decomposition is:\n" << svd.solve(b).transpose() << std::endl;
        std::cout << "The solution using the SVD decomposition is:\n" << svd.matrixV() << std::endl;
        std::cout << "The solution using the SVD decomposition is:\n" << svd.matrixV().col(3) << std::endl;
        std::cout << "The solution using the SVD decomposition is:\n" << svd.matrixV()(3, 3) << std::endl;
        
        // 解质量不好，放弃
        return true;
    }
    return false;
}

TEST(MyslamTest, Triangulation)
{
    Vec3 pt_world(30, 20, 10), pt_world_estimated;
    std::vector<SE3> poses{
        SE3(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, 0, 0)),
        SE3(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, -10, 0)),
        SE3(Eigen::Quaterniond(0, 0, 0, 1), Vec3(0, 10, 0)),
    };
    std::vector<Vec3> points;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        Vec3 pc = poses[i] * pt_world;
        pc /= pc[2];
        points.push_back(pc);
    }

    EXPECT_TRUE(triangulation(poses, points, pt_world_estimated));
    EXPECT_NEAR(pt_world[0], pt_world_estimated[0], 0.01);
    EXPECT_NEAR(pt_world[1], pt_world_estimated[1], 0.01);
    EXPECT_NEAR(pt_world[2], pt_world_estimated[2], 0.01);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
