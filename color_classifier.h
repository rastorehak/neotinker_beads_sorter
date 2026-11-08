#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

constexpr int COLOR_FEATURE_COUNT = 9;
constexpr float MIN_VISIBLE_SUM = 100.0f;
constexpr float MAX_VALID_SENSOR_COUNT = 18000.0f;
constexpr float BRIGHTNESS_WEIGHT = 0.05f;
constexpr float BRIGHTNESS_REFERENCE = 1000.0f;
constexpr float MAX_CLASSIFICATION_DISTANCE = 0.02f;
constexpr int MIN_VOTE_MARGIN = 3;

constexpr uint32_t TRAINING_FILE_MAGIC = 0x41533431; // AS41
constexpr uint32_t TRAINING_FILE_VERSION = 2;

struct TrainingFileHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t classCount;
    uint32_t sampleCapacity;
    uint32_t featureCount;
};

inline bool trainingHeaderIsCompatible(const TrainingFileHeader &header,
                                       size_t classCount,
                                       size_t sampleCapacity,
                                       size_t featureCount) {
    return header.magic == TRAINING_FILE_MAGIC &&
           header.version == TRAINING_FILE_VERSION &&
           header.classCount == classCount &&
           header.sampleCapacity == sampleCapacity &&
           header.featureCount == featureCount;
}

inline bool trainingSampleCountsAreValid(const int *sampleCount,
                                         size_t classCount,
                                         int sampleCapacity) {
    for (size_t c = 0; c < classCount; c++) {
        if (sampleCount[c] < 0 || sampleCount[c] > sampleCapacity) return false;
    }
    return true;
}

inline size_t trainingFileSize(size_t headerSize,
                               size_t trainingDataSize,
                               size_t sampleCountSize) {
    return headerSize + trainingDataSize + sampleCountSize;
}

inline bool trainingFileSizeIsValid(size_t actualSize,
                                    size_t headerSize,
                                    size_t trainingDataSize,
                                    size_t sampleCountSize) {
    return actualSize == trainingFileSize(headerSize, trainingDataSize, sampleCountSize);
}

inline bool normalizeSpectrum(const float *input, float *output) {
    float visibleSum = 0.0f;
    for (int i = 0; i < 8; i++) {
        if (input[i] >= MAX_VALID_SENSOR_COUNT) return false;
        visibleSum += input[i];
    }

    if (input[8] >= MAX_VALID_SENSOR_COUNT || input[9] >= MAX_VALID_SENSOR_COUNT ||
        visibleSum < MIN_VISIBLE_SUM) {
        return false;
    }

    for (int i = 0; i < 8; i++) output[i] = input[i] / visibleSum;
    output[8] = BRIGHTNESS_WEIGHT * logf(visibleSum / BRIGHTNESS_REFERENCE);
    return true;
}

template <size_t Classes, size_t Samples, size_t Features>
int classifySpectrum(const float *input,
                     const float (&trainingData)[Classes][Samples][Features],
                     const int (&sampleCount)[Classes]) {
    constexpr int K = 6;
    const float voteWeights[K] = {3.0f, 2.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    float kDist[K];
    int kClass[K];
    for (int i = 0; i < K; i++) {
        kDist[i] = INFINITY;
        kClass[i] = -1;
    }

    for (size_t c = 0; c < Classes; c++) {
        int count = sampleCount[c];
        if (count < 0) continue;
        if (count > static_cast<int>(Samples)) count = static_cast<int>(Samples);
        for (int s = 0; s < count; s++) {
            float squaredDistance = 0.0f;
            for (size_t f = 0; f < Features; f++) {
                const float difference = input[f] - trainingData[c][s][f];
                squaredDistance += difference * difference;
            }
            const float distance = sqrtf(squaredDistance);
            if (distance < kDist[K - 1]) {
                kDist[K - 1] = distance;
                kClass[K - 1] = static_cast<int>(c);
                for (int i = K - 1; i > 0 && kDist[i] < kDist[i - 1]; i--) {
                    const float distanceSwap = kDist[i];
                    kDist[i] = kDist[i - 1];
                    kDist[i - 1] = distanceSwap;
                    const int classSwap = kClass[i];
                    kClass[i] = kClass[i - 1];
                    kClass[i - 1] = classSwap;
                }
            }
        }
    }

    if (kClass[0] < 0 || kDist[0] > MAX_CLASSIFICATION_DISTANCE) {
        return static_cast<int>(Classes);
    }

    float votes[Classes] = {};
    for (int i = 0; i < K; i++) {
        if (kClass[i] >= 0) votes[kClass[i]] += voteWeights[i];
    }

    int winner = -1;
    float bestVotes = 0.0f;
    float secondBestVotes = 0.0f;
    for (size_t c = 0; c < Classes; c++) {
        if (votes[c] > bestVotes) {
            secondBestVotes = bestVotes;
            bestVotes = votes[c];
            winner = static_cast<int>(c);
        } else if (votes[c] > secondBestVotes) {
            secondBestVotes = votes[c];
        }
    }

    if (winner < 0 || bestVotes < 5.0f || bestVotes - secondBestVotes < MIN_VOTE_MARGIN) {
        return static_cast<int>(Classes);
    }
    return winner;
}