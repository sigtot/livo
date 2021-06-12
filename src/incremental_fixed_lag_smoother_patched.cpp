/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "incremental_fixed_lag_smoother_patched.h"
#include <gtsam/base/debug.h>

void recursiveMarkAffectedKeys(const gtsam::Key& key, const gtsam::ISAM2Clique::shared_ptr& clique,
                               std::set<gtsam::Key>& additionalKeys)
{
  // Check if the separator keys of the current clique contain the specified key
  if (std::find(clique->conditional()->beginParents(), clique->conditional()->endParents(), key) !=
      clique->conditional()->endParents())
  {
    // Mark the frontal keys of the current clique
    for (gtsam::Key i : clique->conditional()->frontals())
    {
      additionalKeys.insert(i);
    }

    // Recursively mark all of the children
    for (const gtsam::ISAM2Clique::shared_ptr& child : clique->children)
    {
      recursiveMarkAffectedKeys(key, child, additionalKeys);
    }
  }
  // If the key was not found in the separator/parents, then none of its children can have it either
}

gtsam::FixedLagSmoother::Result IncrementalFixedLagSmootherPatched::update(
    const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const KeyTimestampMap& timestamps,
    const boost::optional<gtsam::FastMap<gtsam::FactorIndex, gtsam::FastSet<gtsam::Key>>>& newAffectedKeys,
    const gtsam::FactorIndices& factorsToRemove)
{
  const bool debug = ISDEBUG("IncrementalFixedLagSmoother update");

  gtsam::FastVector<size_t> removedFactors;
  boost::optional<gtsam::FastMap<gtsam::Key, int>> constrainedKeys = boost::none;

  // Update the Timestamps associated with the factor keys
  updateKeyTimestampMap(timestamps);

  // Get current timestamp
  double current_timestamp = getCurrentTimestamp();

  if (debug)
    std::cout << "Current Timestamp: " << current_timestamp << std::endl;

  // Find the set of variables to be marginalized out
  gtsam::KeyVector marginalizableKeys = findKeysBefore(current_timestamp - smootherLag_);

  if (debug)
  {
    std::cout << "Marginalizable Keys: ";
    for (gtsam::Key key : marginalizableKeys)
    {
      std::cout << gtsam::DefaultKeyFormatter(key) << " ";
    }
    std::cout << std::endl;
  }

  // Force iSAM2 to put the marginalizable variables at the beginning
  createOrderingConstraints(marginalizableKeys, constrainedKeys);

  if (debug)
  {
    std::cout << "Constrained Keys: ";
    if (constrainedKeys)
    {
      for (gtsam::FastMap<gtsam::Key, int>::const_iterator iter = constrainedKeys->begin();
           iter != constrainedKeys->end(); ++iter)
      {
        std::cout << gtsam::DefaultKeyFormatter(iter->first) << "(" << iter->second << ")  ";
      }
    }
    std::cout << std::endl;
  }

  // Mark additional keys between the marginalized keys and the leaves
  std::set<gtsam::Key> additionalKeys;
  for (gtsam::Key key : marginalizableKeys)
  {
    gtsam::ISAM2Clique::shared_ptr clique = isam_[key];
    for (const gtsam::ISAM2Clique::shared_ptr& child : clique->children)
    {
      recursiveMarkAffectedKeys(key, child, additionalKeys);
    }
  }
  gtsam::KeyList additionalMarkedKeys(additionalKeys.begin(), additionalKeys.end());

  // Update iSAM2
  gtsam::ISAM2UpdateParams params;
  params.constrainedKeys = constrainedKeys;
  params.removeFactorIndices = factorsToRemove;
  params.constrainedKeys = constrainedKeys;
  params.extraReelimKeys = additionalMarkedKeys;
  params.newAffectedKeys = newAffectedKeys;
  try {
    isamResult_ = isam_.update(newFactors, newTheta, params);
  }
  catch (gtsam::IndeterminantLinearSystemException& e)
  {
    /* Comment this in to print factors and their jacobians
    auto graph = isam_.getFactorsUnsafe();
    auto lin_point = isam_.getLinearizationPoint();

    for (const auto& x : graph) {
      std::cout << "================= Factor =================" << std::endl;
      x->printKeys();

      auto jac = x->linearize(lin_point)->jacobian();
      auto A_i = jac.first;
      auto b_i = jac.second;

      std::cout << "A_i = " << std::endl;
      std::cout << A_i << std::endl;
      std::cout << "b_i = " << std::endl;
      std::cout << b_i << std::endl;
    }
     */

    throw e;
  }

  // Marginalize out any needed variables
  if (marginalizableKeys.size() > 0)
  {
    gtsam::FastList<gtsam::Key> leafKeys(marginalizableKeys.begin(), marginalizableKeys.end());
    isam_.marginalizeLeaves(leafKeys);
  }

  // Remove marginalized keys from the KeyTimestampMap
  eraseKeyTimestampMap(marginalizableKeys);

  // TODO: Fill in result structure
  Result result;
  result.iterations = 1;
  result.linearVariables = 0;
  result.nonlinearVariables = 0;
  result.error = 0;

  if (debug)
    std::cout << "IncrementalFixedLagSmoother::update() Finish" << std::endl;

  return result;
}

bool IncrementalFixedLagSmootherPatched::valueExists(gtsam::Key key)
{
  return isam_.valueExists(key);
}
