#include <gtest/gtest.h>
#include <include/common_include.h>

/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */
inline bool triangulation(SE3 &pose_reference, SE3 &pose_current, Vec3 &point_reference, Vec3 &point_current, Vec3 &point_world)
{
    Mat44 A;
    Vec4 b;
    b.setZero();
    auto m = pose_reference.matrix3x4();
    A.row(0) = point_reference[1] * pose_reference.matrix3x4().row(2) - pose_reference.matrix3x4().row(1);
    A.row(1) = point_reference[0] * pose_reference.matrix3x4().row(2) - pose_reference.matrix3x4().row(0);
    A.row(2) = point_current[1] * pose_current.matrix3x4().row(2) - pose_current.matrix3x4().row(1);
    A.row(3) = point_current[0] * pose_current.matrix3x4().row(2) - pose_current.matrix3x4().row(0);
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    point_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
    {
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
    };
    std::vector<Vec3> points;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        Vec3 pc = poses[i] * pt_world;
        pc /= pc[2];
        points.push_back(pc);
    }

    EXPECT_TRUE(triangulation(poses[0], poses[1], points[0], points[1], pt_world_estimated));
    EXPECT_NEAR(pt_world[0], pt_world_estimated[0], 0.01);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
