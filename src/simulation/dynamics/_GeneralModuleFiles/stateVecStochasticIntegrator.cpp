#include <stdexcept>

#include "stateVecStochasticIntegrator.h"
#include "dynamicObject.h"

std::vector<StateIdToIndexMap>
StateVecStochasticIntegrator::getStateIdToNoiseIndexMaps()
{
    std::vector<StateIdToIndexMap> stateIdToNoiseIndexMaps;

    std::unordered_map<size_t, size_t> sharedNoiseIndexToVectorIndex;

    auto getMapForNoise = [&](size_t dynIndex, const std::string& stateName, size_t noiseIndex)
        -> StateIdToIndexMap&
    {
        std::pair<std::string, size_t> extendedNoiseId = {stateName, noiseIndex};

        // This noise source is not shared with any other state
        if (dynPtrs.at(dynIndex)->dynManager.sharedNoiseMap.count(extendedNoiseId) == 0)
        {
            return stateIdToNoiseIndexMaps.emplace_back();
        }

        // States that share a noise source have the same value
        // at the sharedNoiseMap
        size_t sharedNoiseUUID = dynPtrs.at(dynIndex)->dynManager.sharedNoiseMap.at(extendedNoiseId);

        // This noise source is shared, but this is the first time we
        // generate the map for this noise source
        if (sharedNoiseIndexToVectorIndex.count(sharedNoiseUUID) == 0)
        {
            // register that this sharedNoiseUUID corresponds to the
            // map at this position in the diffusions vector
            // and the same for the stateIdToNoiseIndexMaps
            sharedNoiseIndexToVectorIndex[sharedNoiseUUID] = stateIdToNoiseIndexMaps.size();
            return stateIdToNoiseIndexMaps.emplace_back();
        }

        // This noise source is shared, and the diffusion should be
        // placed at an map that was already created
        size_t vecIndex = sharedNoiseIndexToVectorIndex.at(sharedNoiseUUID);
        return stateIdToNoiseIndexMaps.at(vecIndex);
    };

    for (size_t dynIndex = 0; dynIndex < dynPtrs.size(); dynIndex++) {
        for (auto&& [stateName, stateData] :
             dynPtrs.at(dynIndex)->dynManager.stateContainer.stateMap) {

            for (size_t noiseIndex = 0; noiseIndex < stateData->getNumNoiseSources(); noiseIndex++)
            {
                getMapForNoise(dynIndex, stateName, noiseIndex)[{dynIndex, stateName}] = noiseIndex;
            }
        }
    }

    return stateIdToNoiseIndexMaps;
}

void
StateVecStochasticIntegrator::propagateState(
  double timeStep,
  const Eigen::VectorXd& pseudoTimeSteps,
  const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps
)
{
    // This map will hold the vector of pseudoTimeSteps to use
    // for each state. If a state has 2 sources of noise, this
    // map will hold a vector of size 2 with the value of the pseudo
    // time step used for each time step
    std::vector< std::unordered_map<std::string, std::vector<double>> > localPseudoTimeSteps{dynPtrs.size()};

    auto getLocalPseudoTForState = [&](size_t dynPtrIndex, const std::string& state) -> std::vector<double>&
    {
        if (localPseudoTimeSteps.at(dynPtrIndex).count(state) > 0) return localPseudoTimeSteps.at(dynPtrIndex).at(state);
        auto nNoise = dynPtrs.at(dynPtrIndex)->dynManager.getStateObject(state)->getNumNoiseSources();
        return localPseudoTimeSteps.at(dynPtrIndex).emplace(state, nNoise).first->second;
    };

    // stateIdToNoiseIndexMaps is used to map between generated
    // sources of noise and the ones used locally for each state data
    for (size_t k = 0; k < stateIdToNoiseIndexMaps.size(); k++)
    {
        for (auto&& [stateId, noiseIndex] : stateIdToNoiseIndexMaps.at(k))
        {
            getLocalPseudoTForState(stateId.first, stateId.second).at(noiseIndex) = pseudoTimeSteps[k];
        }
    }

    for (size_t dynPtrIndex = 0; dynPtrIndex < dynPtrs.size(); dynPtrIndex++)
    {
        this->dynPtrs.at(dynPtrIndex)->dynManager.propagateStateVector(timeStep, localPseudoTimeSteps.at(dynPtrIndex));
    }
}
