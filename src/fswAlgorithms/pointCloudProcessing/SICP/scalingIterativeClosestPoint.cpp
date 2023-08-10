/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*/

#include "scalingIterativeClosestPoint.h"

ScalingIterativeClosestPoint::ScalingIterativeClosestPoint() = default;

ScalingIterativeClosestPoint::~ScalingIterativeClosestPoint() = default;

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states
 * between function calls are reset to their default values.
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void ScalingIterativeClosestPoint::Reset(uint64_t CurrentSimNanos)
{
    if (!this->measuredPointCloud.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Measured Point Cloud wasn't connected.");
    }
    if (!this->referencePointCloud.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Measured Point Cloud wasn't connected.");
    }
    if (!this->initialCondition.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Initial condition message wasn't connected.");
    }
}

/*! Compute the closest reference point to each measured point
 @return void
 @param Eigen::MatrixXd R_kmin1 : The current rotation to apply to the measured cloud
 @param Eigen::MatrixXd t_kmin1 : The current translation to apply to the measured cloud
 @param Eigen::MatrixXd s_kmin1 : The current scale to apply to the measured cloud
 @param Eigen::MatrixXd measuredPoints : The measured point cloud
 @param Eigen::MatrixXd referencePoints : The reference point cloud
 */
void ScalingIterativeClosestPoint::computePointCorrespondance(const Eigen::MatrixXd& R_kmin1,
                                                              const Eigen::MatrixXd& t_kmin1,
                                                              const double s_kmin1,
                                                              const Eigen::MatrixXd& measuredPoints,
                                                              const Eigen::MatrixXd& referencePoints){

    this->correspondingPoints.setZero(POINT_DIM, this->Np);
    for (int i = 0; i < this->Np; i++) {
        Eigen::Index index;
        Eigen::VectorXd displacedMeasuredPoint = Eigen::VectorXd::Zero(POINT_DIM);
        displacedMeasuredPoint = (s_kmin1 * (R_kmin1 * measuredPoints.col(i)) + t_kmin1);

        (referencePoints.colwise() - displacedMeasuredPoint).colwise().squaredNorm().minCoeff(&index);
        this->correspondingPoints.col(i) = referencePoints.col(index);
    }

}

/*! Center point cloud around the average point
 @return void
 @param Eigen::MatrixXd measuredPoints : The measured point cloud
 @param Eigen::MatrixXd referencePoints : The reference point cloud
 */
void ScalingIterativeClosestPoint::centerCloud(const Eigen::MatrixXd& measuredPoints){

    this->q.setZero(POINT_DIM, this->Np);
    this->n.setZero(POINT_DIM, this->Np);
    Eigen::VectorXd measAvg = Eigen::VectorXd::Zero(POINT_DIM);
    Eigen::VectorXd refAvg = Eigen::VectorXd::Zero(POINT_DIM);

    for (int i=0; i < this->Np; i++) {
        measAvg += measuredPoints.col(i);
        refAvg +=  this->correspondingPoints.col(i);
    }
    measAvg /= this->Np;
    refAvg /= this->Np;

    for (int i=0; i < this->Np; i++) {
        this->q.col(i) = measuredPoints.col(i) - measAvg;
        this->n.col(i) = this->correspondingPoints.col(i) - refAvg;
    }
}

/*! Rotation computation for a specific scale
@return Eigen::MatrixXd Rk : the rotation output by SVD
@param double s_kmin1 : The scale factor to use for the R computation
@param double R_kmin1 : The previous rotation
*/
Eigen::MatrixXd ScalingIterativeClosestPoint::computeRk(const double s_kmin1, const Eigen::MatrixXd& R_kmin1){

    Eigen::MatrixXd R_k = Eigen::MatrixXd::Identity(POINT_DIM, POINT_DIM);
    Eigen::MatrixXd H(POINT_DIM, POINT_DIM);

    H.setZero();
    for (int i=0; i < this->Np; i++) {
        H += this->q.col(i) * this->n.col(i).transpose();
    }
    H *= s_kmin1/this->Np;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd Umat = svd.matrixU();
    Eigen::MatrixXd Vmat = svd.matrixV();

    Eigen::MatrixXd Rotation = Vmat * Umat.transpose();

    if (std::abs(Rotation.determinant() - 1) < this->errorTolerance){
        R_k = Rotation;
    }
    else if (std::abs(Rotation.determinant() + 1) < this->errorTolerance){
        std::vector<Eigen::Index> idxs;
        for(size_t i=0; i<svd.singularValues().size(); i++) {
            if (std::abs(svd.singularValues()(i)) < this->errorTolerance) {
                idxs.push_back(i);
            }
        }
        if (idxs.size() == 1){
            Eigen::MatrixXd Ireflection = Eigen::MatrixXd::Identity(POINT_DIM, POINT_DIM);
            Ireflection(idxs.at(0), idxs.at(0)) = -1;
            R_k = Vmat  * Ireflection * Umat.transpose();
        }
        else{
            R_k = R_kmin1;
        }
    }
    return R_k;
}

/*! Scale factor computation for a specific rotation
@return double sk : the scale factor output
@param double R_kmin1 : The rotation factor to use for the scale computation
@param double s_kmin1 : The previous scale factor
*/
double ScalingIterativeClosestPoint::computeSk(const Eigen::MatrixXd& R_kmin1){

    double s_kn;
    double sumNumerator = 0;
    double sumDenominator = 0;
    Eigen::VectorXd numerator_vec;
    Eigen::VectorXd denominator_vec;
    for (int k=0; k< this->Np; k++){
        numerator_vec = R_kmin1*q.col(k);
        sumNumerator += n.col(k).transpose()*numerator_vec;
        denominator_vec = q.col(k);
        sumDenominator += denominator_vec.transpose()*denominator_vec;
    }
    std::vector<double> costFunction;
    for(int m=0; m< this->Np + 1; m++){
        double s = this->scalingMin +
                   (double)m/this->numberScalePoints*(this->scalingMax - this->scalingMin);
        costFunction.push_back(std::abs(s - sumNumerator/sumDenominator));
    }
    std::vector<double>::iterator min_element =
            std::min_element(costFunction.begin(), costFunction.end());
    s_kn = *min_element + sumNumerator/sumDenominator;

    return s_kn;
}

/*! Translation computation for a scale and rotation pair
@return Eigen::MatrixXd Rk : the translation output
@param double s_k : The scale factor to use for the t computation
@param double R_k : The rotation to use for the t computation
@param Eigen::MatrixXd measuredPoints : The measured point cloud
*/
Eigen::MatrixXd ScalingIterativeClosestPoint::computeTk(const double s_k, const Eigen::MatrixXd& R_k,
                                                        const Eigen::MatrixXd& measuredPoints){
    Eigen::VectorXd tkSum1 = Eigen::VectorXd::Zero(POINT_DIM);
    Eigen::VectorXd tkSum2 = Eigen::VectorXd::Zero(POINT_DIM);
    for (int i=0; i < this->Np; i++) {
        tkSum1 += this->correspondingPoints.col(i);
        tkSum2 += s_k * R_k * measuredPoints.col(i);
    }
    tkSum1 /= this->Np;
    tkSum2 /= this->Np;
    return tkSum1 - tkSum2;
}


/*! This module reads the data and checks for validity. If new information is present it
 @return void
 @param CurrentSimNanos The clock time at which the function was called (nanoseconds)
 */
void ScalingIterativeClosestPoint::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Read input messages and zero output messages
    this->outputCloudBuffer = this->outputPointCloud.zeroMsgPayload;
    this->sicpBuffer = this->outputSICPData.zeroMsgPayload;

    this->measuredCloudBuffer = this->measuredPointCloud();
    this->referenceCloudBuffer = this->referencePointCloud();
    bool initicalConditionValidity = false;
    if (this->initialCondition.isLinked()) {
        this->initialConditionBuffer = this->initialCondition();
        initicalConditionValidity = this->initialConditionBuffer.valid;
    }

    //! - If initial condition message exists populate the initial conditions, otherwise use defaults
    if (initicalConditionValidity) {
        this->R_init = cArray2EigenMatrixXd(this->initialConditionBuffer.rotationMatrix,
                                            POINT_DIM,
                                            POINT_DIM);
        this->t_init = Eigen::Map<Eigen::VectorXd>(this->initialConditionBuffer.translation,
                                                   POINT_DIM,
                                                   1);
        this->s_init = this->initialConditionBuffer.scaleFactor[0];
    }
    else {
        this->R_init = Eigen::MatrixXd::Identity(POINT_DIM, POINT_DIM);
        this->t_init = Eigen::VectorXd::Zero(POINT_DIM);
        this->s_init = 1;
    }

    //! - Check for data validity
    if (!this->measuredCloudBuffer.valid) {
        //! - Write the algorithm output data with zeros are results
        this->outputSICPData.write(&this->sicpBuffer, this->moduleID, CurrentSimNanos);
        this->outputPointCloud.write(&this->outputCloudBuffer, this->moduleID, CurrentSimNanos);
    }
    else {
        Eigen::MatrixXd measuredPoints = cArray2EigenMatrixXd(this->measuredCloudBuffer.points,
                                                              POINT_DIM,
                                                              this->measuredCloudBuffer.numberOfPoints);
        Eigen::MatrixXd referencePoints = cArray2EigenMatrixXd(this->referenceCloudBuffer.points,
                                                               POINT_DIM,
                                                               this->referenceCloudBuffer.numberOfPoints);
        //! - Initialize R (rotation matrix), t (translation vector) and s (scale factor).
        //! k and kmin1 refer to the iteration
        Eigen::MatrixXd R_k = Eigen::MatrixXd::Identity(POINT_DIM, POINT_DIM);
        Eigen::MatrixXd t_k = Eigen::VectorXd::Zero(POINT_DIM);
        double s_k = 1;
        Eigen::MatrixXd R_kmin1 = this->R_init;
        Eigen::MatrixXd t_kmin1 = this->t_init;
        double s_kmin1 = this->s_init;

        //! Set number of measured points with same notation as reference document
        this->Np = this->measuredCloudBuffer.numberOfPoints;

        //! - Begin algorithm iterations
        for (int iteration = 0; iteration < this->maxIterations; iteration++) {
            //! - Point correspondance eq (9)
            this->computePointCorrespondance(R_kmin1, t_kmin1, s_kmin1, measuredPoints, referencePoints);

            //! - Center point clouds about average point
            this->centerCloud(measuredPoints);

            for (int iterRS = 0; iterRS < this->maxInternalIterations; iterRS++) {
                //! - Eq 14-17 to find R
                R_k = this->computeRk(s_kmin1, R_kmin1);

                //! - Eq 20-21 to find S (in this case s is a scalar). s_kn is the previous scale in this
                double s_kn = this->computeSk(R_kmin1);
                //! - If the scale factor converges, exit the loop before the maximum iteration is reached
                if (iterRS > 0 && std::abs(s_kn - s_kmin1) < this->errorTolerance) {
                    s_k = s_kn;
                    break;
                }
                s_kmin1 = s_kn;
            }

            //! - Eq 22 to find t
            t_k = this->computeTk(s_k, R_k, measuredPoints);

            //! - Check for method convergence to exit loop, otherwise continue iterating
            if (std::abs(s_k - s_kmin1) < this->errorTolerance &&
                (R_k - R_kmin1).norm() < this->errorTolerance &&
                (t_k - t_kmin1).norm() < this->errorTolerance) {
                break;
            }
            //! - Save intermediate algorithm data
            this->sicpBuffer.scaleFactor[iteration] = s_k;
            eigenMatrixXd2CArray(R_k, &this->sicpBuffer.rotationMatrix[iteration * POINT_DIM * POINT_DIM]);
            eigenMatrixXd2CArray(t_k, &this->sicpBuffer.translation[iteration * POINT_DIM]);
            this->sicpBuffer.numberOfIteration += 1;

            R_kmin1 = R_k;
            s_kmin1 = s_k;
            t_kmin1 = t_k;
        }

        //! - Save the algorithm output data if the measurement was valid
        this->outputCloudBuffer.valid = true;
        this->sicpBuffer.valid = true;
        this->sicpBuffer.timeTag = CurrentSimNanos;
        Eigen::MatrixXd newPoints = Eigen::MatrixXd::Zero(measuredPoints.rows(), measuredPoints.cols());
        for (int i = 0; i < this->Np; i++) {
            newPoints.col(i) = s_k * (R_k * measuredPoints.col(i)) + t_k;
        }
        eigenMatrixXd2CArray(newPoints, this->outputCloudBuffer.points);
        this->outputCloudBuffer.numberOfPoints = this->Np;
        this->outputCloudBuffer.timeTag = CurrentSimNanos;
        //! - Write the algorithm output data with zeros are results
        this->outputSICPData.write(&this->sicpBuffer, this->moduleID, CurrentSimNanos);
        this->outputPointCloud.write(&this->outputCloudBuffer, this->moduleID, CurrentSimNanos);
    }
}
