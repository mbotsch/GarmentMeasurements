#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Dense>

/// Principal Component Analysis
class PCA
{
public:
    /// default constructor
    PCA() : print_statistics_(true) {}

    /// load PCA model
    bool load(const char* filename);

    /// save PCA model
    bool save(const char* filename);

    /// perform PCA on training data, specify how many components to use
    bool train(std::vector<Eigen::VectorXd>& data, const int components);
    bool train(Eigen::MatrixXd& data, const int components);

    /// return mean vector
    Eigen::VectorXd mean() const { return pca_mean_; }

    /// evaluate PCA with parameter set
    Eigen::VectorXd evaluate(const Eigen::VectorXd& _parameters);

    Eigen::VectorXd scale_parameters(const Eigen::VectorXd& _parameters);
    Eigen::VectorXd inv_scale_parameters(const Eigen::VectorXd& _parameters);

    /// dimension of the vector space
    unsigned int dimension() const { return pca_matrix_.rows(); }

    /// number of principal components
    unsigned int components() const { return pca_matrix_.cols(); }

    Eigen::MatrixXd& pca_matrix() { return pca_matrix_; }

    void set_print_statistics(bool print_statistics) { print_statistics_ = print_statistics; }

    void write_statistics(const std::string& filename);

    void compute_parameters(std::vector<Eigen::VectorXd>* data,
                             Eigen::MatrixXd& pca_params,
                             bool scale_by_eigenvalues,
                             bool already_mean_centered);

private:
    // PCA model matrix
    Eigen::MatrixXd pca_matrix_ = {};

    // PCA eigenvalues
    Eigen::VectorXd eigenvalues_ = {};

    // PCA mean vector
    Eigen::VectorXd pca_mean_ = {};

    std::vector<float> coverage_statistics_;

    bool print_statistics_;
};
