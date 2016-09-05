#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>

#define NUMERIAL_ERROR_THRESHOLD	1.0E-6
#define CUBOID_SURFACE_SAMPLING_RANDOM_SEED	20130923

const float INF_POTENTIAL = 1.0e8;
const float DUMMY_POTENTIAL = 1.0e4;
const float MIN_POINT_CONFIDENCE = 0.7;
const float SPARSE_NEIGHBOR_DISTANCE = 0.05;
const float FEATURE_THREAD_FLOAT_INF = 100.0;

const double PI = 3.141592653589793;
const double param_null_cuboid_probability = 0.1;
const double param_sparse_neighbor_distance = 0.05;
const double opt_single_energy_term_weight = 1.0e4;
const double opt_symmetry_energy_term_weight = 1.0e6;
const double MIN_CUBOID_SIZE = 1.0e-12;
const double param_occlusion_test_neighbor_distance = 0.01;

const int NUM_POINT_FEATURE_THREAD = 8;
const int NUM_FEATURE_THREAD = 6;
const int POINT_FEATURES_DIMEN = 27;
const int NULL_LABEL = 9294;
const int POINT_FEATURE_PARTS = 5;
const int PART_RELATION_DIMEN = 32;
const int param_num_point_neighbors = 8;
const int NUM_OF_SAMPLES = 1000;
const int NULL_PART_INDEX = -1;
const int opt_max_iterations = 5;
const int k_num_attributes = 24;
const int min_num_symmetric_point_pairs = 50;
const int param_min_num_cuboid_sample_points = 10;
const int param_max_inference_iteration = 20;

const bool optimize_individual_reflection_symmetry_group = true;
const bool disable_per_point_classifier_terms = false;
const bool disable_label_smoothness_terms = false;
const bool disable_symmetry_terms = false;
const bool disable_part_relation_terms = false;

using namespace std;

const string label_info_path = "../data/dataset_name/";
const string label_info_filename = "regions.txt";
const string label_symmetry_info_filename = "regions_symmetry.txt";
const string symmetry_group_info_filename = "symmetry_groups.txt";
const string training_dir = "../data/training";
const string transformation_filename_prefix = "transformation_";
const string feature_filename_prefix = "feature_";

#endif