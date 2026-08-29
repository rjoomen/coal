/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020. Toyota Research Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of CNRS-LAAS and AIST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** @author Damrong Guoy (Damrong.Guoy@tri.global) */

/** Tests the dynamic axis-aligned bounding box tree.*/

#define BOOST_TEST_MODULE COAL_BROADPHASE_DYNAMIC_AABB_TREE
#include <boost/test/included/unit_test.hpp>

// #include "coal/data_types.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/broadphase/broadphase_dynamic_AABB_tree.h"
#include "coal/broadphase/broadphase_bruteforce.h"
#include "coal/collision.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <set>
#include <utility>
#include <vector>

using namespace coal;

// Pack the data for callback function.
struct CallBackData {
  bool expect_object0_then_object1;
  std::vector<CollisionObject*>* objects;
};

// This callback function tests the order of the two collision objects from
// the dynamic tree against the `data`. We assume that the first two
// parameters are always objects[0] and objects[1] in two possible orders,
// so we can safely ignore the second parameter. We do not use the last
// Scalar& parameter, which specifies the distance beyond which the
// pair of objects will be skipped.

struct DistanceCallBackDerived : DistanceCallBackBase {
  bool distance(CollisionObject* o1, CollisionObject* o2, Scalar& dist) {
    return distance_callback(o1, o2, &data, dist);
  }

  bool distance_callback(CollisionObject* a, CollisionObject*,
                         void* callback_data, Scalar&) {
    // Unpack the data.
    CallBackData* data = static_cast<CallBackData*>(callback_data);
    const std::vector<CollisionObject*>& objects = *(data->objects);
    const bool object0_first = a == objects[0];
    BOOST_CHECK_EQUAL(data->expect_object0_then_object1, object0_first);
    // TODO(DamrongGuoy): Remove the statement below when we solve the
    //  repeatability problem as mentioned in:
    //  https://github.com/flexible-collision-library/fcl/issues/368
    // Expect to switch the order next time.
    data->expect_object0_then_object1 = !data->expect_object0_then_object1;
    // Return true to stop the tree traversal.
    return true;
  }

  CallBackData data;
};

// Tests repeatability of a dynamic tree of two spheres when we call update()
// and distance() again and again without changing the poses of the objects.
// We only use the distance() method to invoke a hierarchy traversal.
// The distance-callback function in this test does not compute the signed
// distance between the two objects; it only checks their order.
//
// Currently every call to update() switches the order of the two objects.
// TODO(DamrongGuoy): Remove the above comment when we solve the
//  repeatability problem as mentioned in:
//  https://github.com/flexible-collision-library/fcl/issues/368
//
BOOST_AUTO_TEST_CASE(DynamicAABBTreeCollisionManager_class) {
  CollisionGeometryPtr_t sphere0 = make_shared<Sphere>(0.1);
  CollisionGeometryPtr_t sphere1 = make_shared<Sphere>(0.2);
  CollisionObject object0(sphere0);
  CollisionObject object1(sphere1);
  const Vec3s position0(Scalar(0.1), Scalar(0.2), Scalar(0.3));
  const Vec3s position1(Scalar(0.11), Scalar(0.21), Scalar(0.31));

  // We will use `objects` to check the order of the two collision objects in
  // our callback function.
  //
  // We use std::vector that contains *pointers* to CollisionObject,
  // instead of std::vector that contains CollisionObject's.
  // Previously we used std::vector<CollisionObject>, and it failed the
  // Eigen alignment assertion on Win32. We also tried, without success, the
  // custom allocator:
  //     std::vector<CollisionObject,
  //                 Eigen::aligned_allocator<CollisionObject>>,
  // but some platforms failed to build.
  std::vector<CollisionObject*> objects;
  objects.push_back(&object0);
  objects.push_back(&object1);

  std::vector<Vec3s> positions;
  positions.push_back(position0);
  positions.push_back(position1);

  DynamicAABBTreeCollisionManager dynamic_tree;
  for (size_t i = 0; i < objects.size(); ++i) {
    objects[i]->setTranslation(positions[i]);
    objects[i]->computeAABB();
    dynamic_tree.registerObject(objects[i]);
  }

  DistanceCallBackDerived callback;
  callback.data.expect_object0_then_object1 = false;
  callback.data.objects = &objects;

  // We repeat update() and distance() many times.  Each time, in the
  // callback function, we check the order of the two objects.
  for (int count = 0; count < 8; ++count) {
    dynamic_tree.update();
    dynamic_tree.distance(&callback);
  }
}

// Records, in a canonical order, every pair of objects a manager hands to the
// callback that the narrow phase confirms to be in collision. Two managers over
// the same scene must produce identical sets.
struct CollidingPairRecorder : CollisionCallBackBase {
  explicit CollidingPairRecorder(const std::vector<CollisionObject*>& objects)
      : objects(objects) {}

  bool collide(CollisionObject* o1, CollisionObject* o2) {
    CollisionRequest request;
    CollisionResult result;
    coal::collide(o1, o2, request, result);
    if (result.isCollision()) pairs.insert(std::minmax(index(o1), index(o2)));
    return false;
  }

  size_t index(const CollisionObject* o) const {
    const auto it = std::find(objects.begin(), objects.end(), o);
    BOOST_REQUIRE(it != objects.end());
    return static_cast<size_t>(it - objects.begin());
  }

  const std::vector<CollisionObject*>& objects;
  std::set<std::pair<size_t, size_t>> pairs;
};

// The scene built below starts with `num_halfspaces` halfspaces, followed by
// planes up to `num_planar_shapes`, then bounded convex shapes.
static const size_t num_halfspaces = 3;
static const size_t num_planar_shapes = 6;

// Builds a scene of halfspaces and planes in three orientations, plus a grid of
// bounded convex shapes laid out to straddle and to clear them. No shape is
// exactly tangent to a halfspace or a plane: the broad-phase cull rejects a
// support point lying exactly on the surface, while the narrow phase reports a
// zero-distance contact as a collision, so a tangent placement would make the
// two disagree for reasons unrelated to the traversal.
static std::vector<std::shared_ptr<CollisionObject>> makePlanarScene() {
  std::vector<std::shared_ptr<CollisionObject>> objects;

  for (size_t axis = 0; axis < num_halfspaces; ++axis) {
    Vec3s normal = Vec3s::Zero();
    normal[static_cast<Eigen::Index>(axis)] = 1;
    objects.push_back(std::make_shared<CollisionObject>(
        make_shared<Halfspace>(normal, Scalar(-0.53))));
  }
  for (size_t axis = 0; axis < num_planar_shapes - num_halfspaces; ++axis) {
    Vec3s normal = Vec3s::Zero();
    normal[static_cast<Eigen::Index>(axis)] = 1;
    objects.push_back(std::make_shared<CollisionObject>(
        make_shared<Plane>(normal, Scalar(0.37))));
  }
  BOOST_REQUIRE_EQUAL(objects.size(), num_planar_shapes);

  int count = 0;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -2; iy <= 2; ++iy) {
      CollisionGeometryPtr_t geometry;
      switch (count % 3) {
        case 0:
          geometry = make_shared<Box>(Scalar(0.4), Scalar(0.4), Scalar(0.4));
          break;
        case 1:
          geometry = make_shared<Sphere>(Scalar(0.25));
          break;
        default:
          geometry = make_shared<Capsule>(Scalar(0.15), Scalar(0.5));
          break;
      }
      Transform3s tf = Transform3s::Identity();
      tf.setTranslation(
          Vec3s(Scalar(0.5) * ix, Scalar(0.5) * iy, Scalar(0.3) * count));
      objects.push_back(std::make_shared<CollisionObject>(geometry, tf));
      ++count;
    }
  }
  return objects;
}

// The dynamic tree culls a halfspace or a plane against the other object using
// the geometry itself rather than its unbounded AABB, and skips the broad phase
// entirely when both objects are planar. Neither shortcut may drop a collision
// the naive manager reports.
BOOST_AUTO_TEST_CASE(DynamicAABBTreeCollisionManager_halfspace_and_plane) {
  const std::vector<std::shared_ptr<CollisionObject>> scene = makePlanarScene();
  std::vector<CollisionObject*> objects;
  objects.reserve(scene.size());
  for (const auto& object : scene) objects.push_back(object.get());

  DynamicAABBTreeCollisionManager dynamic_tree;
  dynamic_tree.registerObjects(objects);
  dynamic_tree.setup();

  NaiveCollisionManager naive;
  naive.registerObjects(objects);
  naive.setup();

  CollidingPairRecorder from_tree(objects);
  dynamic_tree.collide(&from_tree);
  CollidingPairRecorder from_naive(objects);
  naive.collide(&from_naive);

  BOOST_CHECK_EQUAL(from_tree.pairs.size(), from_naive.pairs.size());
  BOOST_CHECK(from_tree.pairs == from_naive.pairs);

  // Guard the scene itself: every shortcut must actually be reached, or the
  // comparison above passes without covering any of them.
  size_t planar_planar = 0;
  size_t halfspace_convex = 0;
  size_t plane_convex = 0;
  for (const auto& pair : from_tree.pairs) {
    if (pair.first >= num_planar_shapes) continue;
    if (pair.second < num_planar_shapes)
      ++planar_planar;
    else if (pair.first < num_halfspaces)
      ++halfspace_convex;
    else
      ++plane_convex;
  }
  BOOST_CHECK_GT(planar_planar, size_t(0));
  BOOST_CHECK_GT(halfspace_convex, size_t(0));
  BOOST_CHECK_GT(plane_convex, size_t(0));
}
