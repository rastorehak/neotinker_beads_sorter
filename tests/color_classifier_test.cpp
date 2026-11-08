#include "../color_classifier.h"

#include <assert.h>
#include <math.h>

namespace {

void testNormalization() {
    float raw[10] = {100, 100, 100, 100, 100, 100, 100, 100, 1000, 200};
    float normalized[COLOR_FEATURE_COUNT] = {};
    assert(normalizeSpectrum(raw, normalized));
    for (int i = 0; i < 8; i++) assert(fabsf(normalized[i] - 0.125f) < 1e-6f);
    assert(fabsf(normalized[8] - BRIGHTNESS_WEIGHT * logf(0.8f)) < 1e-6f);

    for (int i = 0; i < 8; i++) raw[i] *= 2.0f;
    float brighter[COLOR_FEATURE_COUNT] = {};
    assert(normalizeSpectrum(raw, brighter));
    for (int i = 0; i < 8; i++) assert(fabsf(brighter[i] - normalized[i]) < 1e-6f);
    assert(fabsf(brighter[8] - normalized[8] - BRIGHTNESS_WEIGHT * logf(2.0f)) < 1e-6f);
}

void testInvalidReadings() {
    float raw[10] = {};
    float normalized[COLOR_FEATURE_COUNT] = {};
    assert(!normalizeSpectrum(raw, normalized));

    for (int i = 0; i < 8; i++) raw[i] = 12.0f;
    assert(!normalizeSpectrum(raw, normalized));

    raw[3] = MAX_VALID_SENSOR_COUNT;
    assert(!normalizeSpectrum(raw, normalized));
    raw[3] = 12.0f;
    raw[8] = MAX_VALID_SENSOR_COUNT;
    assert(!normalizeSpectrum(raw, normalized));
    raw[8] = 0.0f;
    raw[9] = MAX_VALID_SENSOR_COUNT;
    assert(!normalizeSpectrum(raw, normalized));
}

void testClassificationAndRejection() {
    float training[2][30][COLOR_FEATURE_COUNT] = {};
    int counts[2] = {6, 0};
    float input[COLOR_FEATURE_COUNT] = {0.7f, 0.3f};
    for (int s = 0; s < counts[0]; s++) {
        training[0][s][0] = 0.7f;
        training[0][s][1] = 0.3f;
        training[0][s][8] = s * 0.001f;
    }
    assert(classifySpectrum(input, training, counts) == 0);

    input[0] = 0.2f;
    input[1] = 0.8f;
    assert(classifySpectrum(input, training, counts) == 2);
}

void testBrightnessSeparatesSimilarShapes() {
    float training[2][30][COLOR_FEATURE_COUNT] = {};
    int counts[2] = {6, 6};
    float input[COLOR_FEATURE_COUNT] = {0.7f, 0.3f};
    for (int s = 0; s < 6; s++) {
        training[0][s][0] = training[1][s][0] = 0.7f;
        training[0][s][1] = training[1][s][1] = 0.3f;
        training[1][s][8] = 0.04f;
    }
    input[8] = 0.04f;
    assert(classifySpectrum(input, training, counts) == 1);
}

void testAmbiguousVoteIsRejected() {
    float training[2][30][COLOR_FEATURE_COUNT] = {};
    int counts[2] = {2, 4};
    float input[COLOR_FEATURE_COUNT] = {1.0f};
    for (int s = 0; s < counts[0]; s++) training[0][s][8] = (s + 1) * 0.001f;
    for (int s = 0; s < counts[1]; s++) training[1][s][8] = (s + 3) * 0.001f;
    assert(classifySpectrum(input, training, counts) == 2);
}

void testTrainingFileValidation() {
    const TrainingFileHeader header = {
        TRAINING_FILE_MAGIC, TRAINING_FILE_VERSION, 15, 30, COLOR_FEATURE_COUNT
    };
    assert(trainingHeaderIsCompatible(header, 15, 30, COLOR_FEATURE_COUNT));

    TrainingFileHeader oldHeader = header;
    oldHeader.version--;
    assert(!trainingHeaderIsCompatible(oldHeader, 15, 30, COLOR_FEATURE_COUNT));
    assert(!trainingHeaderIsCompatible(header, 15, 20, COLOR_FEATURE_COUNT));

    const size_t expectedSize = trainingFileSize(sizeof(header), 16200, 60);
    assert(trainingFileSizeIsValid(expectedSize, sizeof(header), 16200, 60));
    assert(!trainingFileSizeIsValid(expectedSize - 1, sizeof(header), 16200, 60));
    assert(!trainingFileSizeIsValid(expectedSize + 1, sizeof(header), 16200, 60));

    int validCounts[3] = {0, 30, 12};
    int invalidCounts[3] = {0, 31, 12};
    int negativeCounts[3] = {0, -1, 12};
    assert(trainingSampleCountsAreValid(validCounts, 3, 30));
    assert(!trainingSampleCountsAreValid(invalidCounts, 3, 30));
    assert(!trainingSampleCountsAreValid(negativeCounts, 3, 30));
}

}

int main() {
    testNormalization();
    testInvalidReadings();
    testClassificationAndRejection();
    testBrightnessSeparatesSimilarShapes();
    testAmbiguousVoteIsRejected();
    testTrainingFileValidation();
}