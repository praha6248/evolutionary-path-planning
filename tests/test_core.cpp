// Unit tests for the path-planning core (Map, Goal_function, Population).
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "goal_function.hpp"
#include "map.hpp"
#include "point.h"
#include "population.hpp"

namespace {

// Writes a map file to a temporary location and returns its path.
std::string write_map(const std::string &name, const std::string &contents) {
    auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream(path) << contents;
    return path.string();
}

// 10 x 5 map with a single vertical wall in column 5 (rows 0-3), gap at the bottom row.
const char *kWallMap =
    "# test map\n"
    "size 10 5\n"
    "start 1.5 1.5\n"
    "goal 8.5 1.5\n"
    ".....#....\n"
    ".....#....\n"
    ".....#....\n"
    ".....#....\n"
    "..........\n";

Map load_wall_map() {
    Map m;
    m.load(write_map("robot_ga_wall.txt", kWallMap));
    return m;
}

}  // namespace

// ---------------------------------------------------------------- Map

TEST(Map, LoadsSizeStartAndGoal) {
    Map m = load_wall_map();
    EXPECT_EQ(m.get_width(), 10);
    EXPECT_EQ(m.get_height(), 5);
    EXPECT_DOUBLE_EQ(m.get_start().xp, 1.5);
    EXPECT_DOUBLE_EQ(m.get_goal().xp, 8.5);
    EXPECT_EQ(m.get_grid().size(), 50u);
}

TEST(Map, IsFreeRespectsObstaclesAndBounds) {
    Map m = load_wall_map();
    EXPECT_TRUE(m.is_free(0.5, 0.5));
    EXPECT_FALSE(m.is_free(5.5, 0.5));   // wall
    EXPECT_TRUE(m.is_free(5.5, 4.5));    // gap under the wall
    EXPECT_FALSE(m.is_free(-0.1, 1.0));  // outside counts as obstacle
    EXPECT_FALSE(m.is_free(10.0, 1.0));
    EXPECT_FALSE(m.is_free(1.0, 5.0));
}

TEST(Map, CollisionLengthAcrossOneCellWall) {
    Map m = load_wall_map();
    // horizontal segment crossing the 1-cell-wide wall
    EXPECT_NEAR(m.collision_length(Point(1.5, 1.5), Point(8.5, 1.5)), 1.0, 0.11);
    // segment through the gap does not collide
    EXPECT_DOUBLE_EQ(m.collision_length(Point(1.5, 4.5), Point(8.5, 4.5)), 0.0);
    // zero-length segment
    EXPECT_DOUBLE_EQ(m.collision_length(Point(2, 2), Point(2, 2)), 0.0);
}

TEST(Map, RejectsInvalidFiles) {
    Map m;
    EXPECT_THROW(m.load("/nonexistent/map.txt"), std::runtime_error);
    EXPECT_THROW(m.load(write_map("robot_ga_badrow.txt",
                                  "size 3 2\nstart 0.5 0.5\ngoal 2.5 0.5\n...\n..\n")),
                 std::runtime_error);
    EXPECT_THROW(m.load(write_map("robot_ga_blocked.txt",
                                  "size 3 1\nstart 0.5 0.5\ngoal 2.5 0.5\n#..\n")),
                 std::runtime_error);
    EXPECT_THROW(m.load(write_map("robot_ga_key.txt", "size 3 1\nfoo 1\n")),
                 std::runtime_error);
}

// ---------------------------------------------------------------- Goal_function

TEST(GoalFunction, LengthIsSumOfSegments) {
    std::vector<Point> path = {Point(0, 0), Point(3, 4), Point(3, 8)};
    Goal_function g(path);
    EXPECT_DOUBLE_EQ(g.length(), 9.0);
    EXPECT_DOUBLE_EQ(g.collision(), 0.0);  // no map -> no collisions
    EXPECT_DOUBLE_EQ(g.value(), 9.0);
}

TEST(GoalFunction, DegeneratePathHasZeroLength) {
    std::vector<Point> one = {Point(1, 1)};
    EXPECT_DOUBLE_EQ(Goal_function(one).length(), 0.0);
}

TEST(GoalFunction, ValueAddsPenalisedCollision) {
    Map m = load_wall_map();
    std::vector<Point> path = {m.get_start(), m.get_goal()};
    Goal_function g(path, m, 10.0);
    EXPECT_NEAR(g.length(), 7.0, 1e-12);
    EXPECT_NEAR(g.value(), g.length() + 10.0 * g.collision(), 1e-12);
    EXPECT_GT(g.collision(), 0.0);
}

// ---------------------------------------------------------------- Population

TEST(Population, SameSeedGivesSameResult) {
    Map m = load_wall_map();
    std::vector<Point> rp = {m.get_start(), m.get_goal()};
    Population a(6, rp, m, 42), b(6, rp, m, 42);
    for (int i = 0; i < 20; i++) { a.next_generation(); b.next_generation(); }
    EXPECT_DOUBLE_EQ(a.best_value(), b.best_value());
}

TEST(Population, ElitismNeverWorsensBestValue) {
    Map m = load_wall_map();
    std::vector<Point> rp = {m.get_start(), m.get_goal()};
    Population pop(6, rp, m, 7);
    double prev = pop.best_value();
    for (int i = 0; i < 100; i++) {
        pop.next_generation();
        ASSERT_LE(pop.best_value(), prev + 1e-9) << "generation " << i;
        prev = pop.best_value();
    }
}

TEST(Population, IndividualsStayValid) {
    Map m = load_wall_map();
    std::vector<Point> rp = {m.get_start(), m.get_goal()};
    const int max_path = 5;
    Parameters p;
    p.pop_size = 51;  // odd size exercises the unpaired-parent branch
    Population pop(max_path, rp, m, 3, p);
    for (int i = 0; i < 50; i++) pop.next_generation();

    EXPECT_EQ(pop.get_population().size(), 51u);
    for (const auto &ind : pop.get_population()) {
        EXPECT_LE((int)ind.size(), max_path);
        for (const auto &pt : ind) {
            EXPECT_GE(pt.xp, 0.0); EXPECT_LT(pt.xp, m.get_width());
            EXPECT_GE(pt.yp, 0.0); EXPECT_LT(pt.yp, m.get_height());
        }
    }
    for (double f : pop.get_fitness()) {
        EXPECT_GT(f, 0.0);
        EXPECT_LE(f, 1.0);
    }
}

TEST(Population, BestPathStartsAtStartAndEndsAtGoal) {
    Map m = load_wall_map();
    std::vector<Point> rp = {m.get_start(), m.get_goal()};
    Population pop(4, rp, m, 1);
    auto path = pop.best_path();
    ASSERT_GE(path.size(), 2u);
    EXPECT_DOUBLE_EQ(path.front().xp, m.get_start().xp);
    EXPECT_DOUBLE_EQ(path.back().xp, m.get_goal().xp);
}

TEST(Population, FindsCollisionFreePathAroundWall) {
    Map m = load_wall_map();
    std::vector<Point> rp = {m.get_start(), m.get_goal()};
    Parameters p;
    p.pop_size = 150;
    Population pop(6, rp, m, 1, p);
    for (int i = 0; i < 300; i++) pop.next_generation();
    EXPECT_DOUBLE_EQ(pop.best_collision(), 0.0);
    // the detour through the gap must be longer than the straight line
    EXPECT_GT(pop.best_value(), 7.0);
}
