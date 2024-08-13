#include "PCA.h"

//#include <cfloat>
#include <fstream>
//#include <iostream>
//#include <string>

#include <pmp/surface_mesh.h>

bool PCA::load(const char* filename)
{
    // open file
    std::ifstream ifs(filename, std::ifstream::binary);
    if (!ifs.is_open())
    {
        std::cerr << "cannot read pca from " << filename << std::endl;
        return false;
    }

    // read data
    unsigned int m, n;
    ifs.read(reinterpret_cast<char *>(&m), sizeof(m));
    ifs.read(reinterpret_cast<char *>(&n), sizeof(n));
    pca_matrix_.resize(m,n);
    pca_mean_.resize(m);
    eigenvalues_.resize(n);
    ifs.read(reinterpret_cast<char *>(pca_matrix_.data()), m*n*sizeof(double));
    ifs.read(reinterpret_cast<char *>(pca_mean_.data()), m*sizeof(double));
    ifs.read(reinterpret_cast<char *>(eigenvalues_.data()), n*sizeof(double));

    // close file
    ifs.close();

    return true;
}

bool PCA::save(const char* filename)
{
    // open file
    std::ofstream ofs(filename, std::ofstream::binary);
    if (!ofs.is_open())
    {
        std::cerr << "cannot write pca\n";
        return false;
    }

    // write data
    unsigned int n = pca_matrix_.cols();
    unsigned int m = pca_matrix_.rows();
    ofs.write(reinterpret_cast<const char *>(&m), sizeof(int));
    ofs.write(reinterpret_cast<const char *>(&n), sizeof(int));
    ofs.write(reinterpret_cast<const char *>(pca_matrix_.data()), m*n*sizeof(double));
    ofs.write(reinterpret_cast<const char *>(pca_mean_.data()), m*sizeof(double));
    ofs.write(reinterpret_cast<const char *>(eigenvalues_.data()), n*sizeof(double));

    // close file
    ofs.close();

    if (print_statistics_)
    {
        write_statistics(std::string(filename) + "_stats.txt");
    }

    return true;
}

void PCA::write_statistics(const std::string& filename)
{
    std::ofstream ofs(filename);
    if (!ofs)
    {
        printf("[ERROR] Cannot open %s for writing\n", filename.c_str());
        return;
    }

    int index = 1;
    for (float val : coverage_statistics_)
    {
        ofs << "Coverage up until component " << index++ << ": " << val << std::endl;
    }
}

void PCA::compute_parameters(std::vector<Eigen::VectorXd>* data,
                             Eigen::MatrixXd& pca_params,
                             bool scale_by_eigenvalues,
                             bool already_mean_centered)
{
    if (print_statistics_)
        std::cout << "Computing parameters for training data ..." << std::endl;

    size_t num_meshes = data->size();
    pca_params.resize(num_meshes, components());

    for (size_t m = 0; m < num_meshes; ++m)
    {
        Eigen::VectorXd points_mesh = data->at(m);
        if (!already_mean_centered)
            points_mesh -= pca_mean_;

        Eigen::VectorXd params_mesh_m = pca_matrix_.transpose() * points_mesh;

        if (scale_by_eigenvalues)
        {
            params_mesh_m = inv_scale_parameters(params_mesh_m);
        }

        pca_params.row(m) = params_mesh_m;
    }
}

#if 0

void PCA::compute_parameters(const std::string& fn_output, std::vector<Eigen::VectorXd>* data,
                             bool scale_by_eigenvalues,
                             bool already_mean_centered)
{
    if (print_statistics_)
        std::cout << "Computing parameters for training data ..." << std::endl;

    Eigen::MatrixXd pca_params;

    size_t num_meshes = data->size();

    double total_error = 0.0;
    double max_error = 0.0;
    int max_error_index = 0;

    pca_params.resize(num_meshes, components());

    double min_param =  DBL_MAX;
    double max_param = -DBL_MAX;

    for (size_t m = 0; m < num_meshes; ++m)
    {
        Eigen::VectorXd points_mesh = data->at(m);

        if (!already_mean_centered)
            points_mesh -= pca_mean_;

        Eigen::VectorXd params_mesh_m = pca_matrix_.transpose() * points_mesh;

        if (scale_by_eigenvalues)
        {
            params_mesh_m = inv_scale_parameters(params_mesh_m);
        }

        min_param = std::min(min_param, params_mesh_m.minCoeff());
        max_param = std::max(max_param, params_mesh_m.maxCoeff());

        pca_params.row(m) = params_mesh_m;

        double error = 0.0;
        Eigen::VectorXd predicted;
        if (scale_by_eigenvalues)
        {
            predicted = pca_matrix_ * scale_parameters(params_mesh_m);
        }
        else
        {
            predicted = pca_matrix_ * params_mesh_m;
        }

        points_mesh += pca_mean_;
        predicted   += pca_mean_;

        int num_points = points_mesh.size() / 3;
        for (int i = 0; i < num_points; ++i)
        {
            auto p_input = pmp::Point(points_mesh(0 * num_points + i),
                                      points_mesh(1 * num_points + i),
                                      points_mesh(2 * num_points + i));
            auto p_predi = pmp::Point(predicted(0 * num_points + i),
                                      predicted(1 * num_points + i),
                                      predicted(2 * num_points + i));
            error += distance(p_input, p_predi);
        }

        error /= num_points;

        if (print_statistics_)
        {
         //   printf("[INFO] Mesh %d: Error %f\n", (int)m, error);
        }

        if (max_error < error)
        {
            max_error = error;
            max_error_index = m;
        }

        total_error += error;

    }

    if (print_statistics_)
    {
        std::cout << "Min PCA Param: " << min_param << std::endl;
        std::cout << "Max PCA Param: " << max_param << std::endl;
        std::cout << "[INFO] Reconstruction Error " << total_error / num_meshes << std::endl;
        printf("[INFO] Max error %f for mesh %d\n", max_error, max_error_index);
        std::cout << "DONE" << std::endl;
    }

    write_eigen_matrix(fn_output, pca_params);
}

#endif

Eigen::VectorXd PCA::scale_parameters(const Eigen::VectorXd& _parameters)
{
    // scale parameters by standard deviation
    Eigen::VectorXd param = _parameters;
    for (unsigned int i=0; i<param.size(); ++i)
    {
        if (eigenvalues_[i] > 1e-13)
        {
            param[i] *= sqrt(eigenvalues_[i]);
        }
        else
        {
            param[i] *= fabs(eigenvalues_[i]);
        }
    }

    return param;
}

Eigen::VectorXd PCA::inv_scale_parameters(const Eigen::VectorXd& _parameters)
{
    // scale parameters by standard deviation
    Eigen::VectorXd param = _parameters;
    for (unsigned int i=0; i<param.size(); ++i)
    {
        if (eigenvalues_[i] > 1e-13)
            param[i] /= sqrt(eigenvalues_[i]);
    }

    return param;
}

Eigen::VectorXd PCA::evaluate(const Eigen::VectorXd& _parameters)
{
    if (_parameters.size() != components())
    {
        std::cerr << "PCA::evaluate: dimension does not fit, only "<< components() << " components! " << "there are " <<_parameters.size() << "\n";
        return pca_mean_;
    }

    // evalute PCA
    return pca_matrix_ * _parameters + pca_mean_;
}

bool PCA::train(std::vector<Eigen::VectorXd>& data, const int ncomponents)
{
    // check input parameters
    if (data.empty())
    {
        std::cerr << "PCA::train: empty data\n";
        return false;
    }
    if (ncomponents<1)
    {
        std::cerr << "PCA::train: bad #components\n";
        return false;
    }

    // n: number of training vectors
    // m: dimension of training vectors
    const unsigned int n = data.size();
    const unsigned int m = data[0].size();

    for (unsigned int i = 0; i<n; ++i)
    {
        if (data[i].size() != m)
        {
            std::cerr << "PCA::train: data dimension does not match\n";
            return false;
        }
    }

    if (print_statistics_)
        std::cout << "[INFO] Compute PCA:\n" << std::flush;

    // compute mean
    pca_mean_ = Eigen::VectorXd::Zero(m);
    for (unsigned int i = 0; i < n; ++i)
    {
        pca_mean_ += data[i];
    }
    pca_mean_ /= n;


    // mean-center data
    for (unsigned int i = 0; i < n; ++i)
    {
        data[i] -= pca_mean_;
    }


    // build data matrix. put training data in matrix columns
    Eigen::MatrixXd X(m,n);
    for (unsigned int col=0; col<n; ++col)
        for (unsigned int row=0; row<m; ++row)
            X(row, col) = data[col][row];
    X /= sqrt(n-1);



#if 0 // Jascha's version

    // build X^T*X
    Eigen::MatrixXd C = X.transpose() * X;

    // implement eigendecomposition through SVD
    Eigen::BDCSVD<Eigen::MatrixXd> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::VectorXd eigenvalues = svd.singularValues();

    // build U, normalize its columns
    Eigen::MatrixXd U = X * V;
    U.colwise().normalize();
#endif

#if 1  // Mario's version

    // build covariance matrix
    Eigen::MatrixXd C = X.transpose() * X;

    // Eigen-decomposition of C
    // have to reverse columns, since eigenvalues are sorted in increasing order
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(C);
    Eigen::VectorXd eigenvalues = solver.eigenvalues().reverse();
    Eigen::MatrixXd V = solver.eigenvectors().rowwise().reverse();

    // build U, normalize its columns
    Eigen::MatrixXd U = X*V;
    U.colwise().normalize();
#endif

#if 0  // less efficient PCA version

    Eigen::BDCSVD<Eigen::MatrixXd> svd(X, Eigen::ComputeThinU);
    Eigen::MatrixXd U = svd.matrixU();
    U.colwise().normalize();
    Eigen::VectorXd sd = svd.singularValues();
    Eigen::VectorXd eigenvalues = sd.array().square();

#endif

    coverage_statistics_.clear();
    // which percentage of training data is explained by how many components?
    double total_variance = eigenvalues.sum();
    double variance(0);
    for (unsigned int k = 0; k < n; ++k)
    {
        variance += eigenvalues[k];
        coverage_statistics_.push_back(variance/total_variance);
        int percentage = (int)(100*variance/total_variance+0.5);

        if (print_statistics_)
            std::cout << "  first " << (k+1) << " components explain " << percentage << "% of variance\n";
        if(percentage >= 100)
        {
            break;
        }
    }

    // how many principal components to retain
    pca_matrix_  = U.leftCols(ncomponents);
    eigenvalues_ = eigenvalues.topRows(ncomponents);

    // output final dimensions for debugging
    if (print_statistics_)
    {
        std::cout << "  PCA matrix is   " << pca_matrix_.rows() << " x " << pca_matrix_.cols() << std::endl;
        std::cout << "  PCA mean is     " << pca_mean_.rows()   << " x " << pca_mean_.cols() << std::endl;
        std::cout << "  Eigenvalues are " << eigenvalues_.transpose() << std::endl;
    }

    return true;
}

bool PCA::train(Eigen::MatrixXd& X, const int ncomponents)
{
    // check input parameters
    if (X.size() == 0)
    {
        std::cerr << "PCA::train: empty data\n";
        return false;
    }
    if (ncomponents<1)
    {
        std::cerr << "PCA::train: bad #components\n";
        return false;
    }

    // n: number of training vectors
    // m: dimension of training vectors
    const unsigned int m = X.cols();
    const unsigned int n = X.rows();

    if (print_statistics_)
    {
        std::cout << "Compute PCA:\n" << std::flush;
    }

    // compute mean
    pca_mean_ = Eigen::VectorXd::Zero(m);
    for (unsigned int i = 0; i < n; ++i)
    {
        pca_mean_ += X.row(i);
    }
    pca_mean_ /= n;

    // mean-center X
    for (unsigned int i = 0; i < n; ++i)
    {
        X.row(i) -= pca_mean_;
    }

    X /= sqrt(n-1);

    X.transposeInPlace();

#if 0 // Jascha's version

    // build X^T*X
    Eigen::MatrixXd C = X.transpose() * X;

    // implement eigendecomposition through SVD
    Eigen::BDCSVD<Eigen::MatrixXd> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::VectorXd eigenvalues = svd.singularValues();

    // build U, normalize its columns
    Eigen::MatrixXd U = X * V;
    U.colwise().normalize();
#endif

#if 1  // Mario's version

    // build covariance matrix
    Eigen::MatrixXd C = X.transpose() * X;

    // Eigen-decomposition of C
    // have to reverse columns, since eigenvalues are sorted in increasing order
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(C);
    Eigen::VectorXd eigenvalues = solver.eigenvalues().reverse();
    Eigen::MatrixXd V = solver.eigenvectors().rowwise().reverse();

    // build U, normalize its columns
    Eigen::MatrixXd U = X*V;
    U.colwise().normalize();
#endif

#if 0  // less efficient PCA version

    Eigen::BDCSVD<Eigen::MatrixXd> svd(X, Eigen::ComputeThinU);
    Eigen::MatrixXd U = svd.matrixU();
    U.colwise().normalize();
    Eigen::VectorXd sd = svd.singularValues();
    Eigen::VectorXd eigenvalues = sd.array().square();

#endif

    // which percentage of training data is explained by how many components?
    coverage_statistics_.clear();
    double total_variance = eigenvalues.sum();
    double variance(0);
    for (unsigned int k = 0; k < n; ++k)
    {
        variance += eigenvalues[k];
        int percentage = (int)(100*variance/total_variance+0.5);
        coverage_statistics_.push_back(variance/total_variance);

        if (print_statistics_)
            std::cout << "  first " << (k+1) << " components explain " << percentage << "% of variance\n";
        if(percentage >= 100)
        {
            break;
        }
    }

    // how many principal components to retain
    pca_matrix_  = U.leftCols(ncomponents);
    eigenvalues_ = eigenvalues.topRows(ncomponents);

    // output final dimensions for debugging
    if (print_statistics_)
    {
        std::cout << "  PCA matrix is   " << pca_matrix_.rows() << " x " << pca_matrix_.cols() << std::endl;
        std::cout << "  PCA mean is     " << pca_mean_.rows()   << " x " << pca_mean_.cols() << std::endl;
        std::cout << "  Eigenvalues are " << eigenvalues_.transpose() << std::endl;
    }

    return true;
}

//=============================================================================
