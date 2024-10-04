/*
    Author: Giorgio Medico
    Date: 24/09/2024
    Description: Classe per la rappresentazione e manipolazione di rotazioni in 3D
    File : rotation_lib.cpp
*/

#include "rotation_lib.h"

namespace MoveG
{

// Implementazione dei metodi della classe Rotation

// Costruttore di default (rotazione identità)
Rotation::Rotation() : q(Eigen::Quaterniond::Identity())
{
}

// Copy constructor
Rotation::Rotation(const Rotation &other) : q(other.q)
{
}

// Costruttore da matrice di rotazione
Rotation::Rotation(const Eigen::Matrix3d &rotation_matrix) : q(Eigen::Quaterniond(rotation_matrix))
{
}

// Costruttore da quaternione
Rotation::Rotation(const Eigen::Quaterniond &quaternion) : q(quaternion.normalized())
{
}

// Costruttore da asse-angolo
Rotation::Rotation(const Eigen::AngleAxisd &angle_axis) : q(Eigen::Quaterniond(angle_axis))
{
}

// Costruttore da angoli di Eulero utilizzando rappresentazioni asse-angolo
Rotation::Rotation(double angle1,
                   double angle2,
                   double angle3,
                   bool intrinsic,
                   const std::string &sequence,
                   bool degree)
    : q(Eigen::Quaterniond::Identity())
{
    checkSequence(sequence);

    if (degree)
    {
        angle1 = deg2rad(angle1);
        angle2 = deg2rad(angle2);
        angle3 = deg2rad(angle3);
    }

    Eigen::AngleAxisd aa1 = angleAxis(sequence[0], angle1);
    Eigen::AngleAxisd aa2 = angleAxis(sequence[1], angle2);
    Eigen::AngleAxisd aa3 = angleAxis(sequence[2], angle3);

    if (intrinsic)
    {
        // Per rotazioni intrinseche, moltiplica nell'ordine q3 * q2 * q1
        q = Eigen::Quaterniond(aa3) * Eigen::Quaterniond(aa2) * Eigen::Quaterniond(aa1);
    }
    else
    {
        // Per rotazioni estrinseche, moltiplica nell'ordine q1 * q2 * q3
        q = Eigen::Quaterniond(aa1) * Eigen::Quaterniond(aa2) * Eigen::Quaterniond(aa3);
    }

    q.normalize();
}

// Metodi statici
Rotation Rotation::fromRotationMatrix(const Eigen::Matrix3d &rotation_matrix)
{
    return Rotation(rotation_matrix);
}

Rotation Rotation::fromQuaternion(const Eigen::Quaterniond &quaternion)
{
    return Rotation(quaternion);
}

Rotation Rotation::fromAngleAxis(const Eigen::AngleAxisd &angle_axis)
{
    return Rotation(angle_axis);
}

Rotation Rotation::fromEulerAngles(double angle1,
                                   double angle2,
                                   double angle3,
                                   bool intrinsic,
                                   const std::string &sequence,
                                   bool degree)
{
    return Rotation(angle1, angle2, angle3, intrinsic, sequence, degree);
}

// Convertitore a matrice di rotazione
Eigen::Matrix3d Rotation::toRotationMatrix() const
{
    return q.toRotationMatrix();
}

// Convertitore a quaternione
Eigen::Quaterniond Rotation::toQuaternion() const
{
    return q;
}

// Convertitore ad asse-angolo
Eigen::AngleAxisd Rotation::toAngleAxis() const
{
    return Eigen::AngleAxisd(q);
}

// Convertitore ad angoli di Eulero
Eigen::Vector3d Rotation::toEulerAngles(bool intrinsic, const std::string &sequence) const
{
    checkSequence(sequence);

    int a0 = axisToIndex(sequence[0]);
    int a1 = axisToIndex(sequence[1]);
    int a2 = axisToIndex(sequence[2]);

    Eigen::Vector3d angles = Eigen::Vector3d::Zero();

    if (intrinsic)
    {
        // Per rotazioni intrinseche, inverti l'ordine degli assi
        angles = q.toRotationMatrix().eulerAngles(a2, a1, a0).reverse();
    }
    else
    {
        // Per rotazioni estrinseche, usa gli assi come sono
        angles = q.toRotationMatrix().eulerAngles(a0, a1, a2);
    }

    return angles;
}

// Sovraccarico dell'operatore * (composizione di rotazioni)
Rotation Rotation::operator*(const Rotation &other) const
{
    return Rotation(q * other.q);
}

Rotation &Rotation::operator=(const Rotation &other)
{
    if (this != &other)
    {
        q = other.q;
    }
    return *this;
}

// Funzione ausiliaria per creare una rappresentazione asse-angolo
Eigen::AngleAxisd Rotation::angleAxis(char axis, double angle)
{
    switch (axis)
    {
    case 'X':
    case 'x':
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
    case 'Y':
    case 'y':
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());
    case 'Z':
    case 'z':
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    default:
        throw std::invalid_argument("Asse non valido. Gli assi validi sono 'X', 'Y' o 'Z'.");
    }
}

// Funzione ausiliaria per convertire l'asse in indice
int Rotation::axisToIndex(char axis)
{
    switch (axis)
    {
    case 'X':
    case 'x':
        return 0;
    case 'Y':
    case 'y':
        return 1;
    case 'Z':
    case 'z':
        return 2;
    default:
        throw std::invalid_argument("Asse non valido. Gli assi validi sono 'X', 'Y' o 'Z'.");
    }
}

// Funzione ausiliaria per controllare la sequenza di angoli di Eulero
int Rotation::checkSequence(const std::string &sequence)
{
    if (sequence.length() != 3 || sequence.find_first_not_of("XYZxyz") != std::string::npos)
    {
        throw std::invalid_argument("La sequenza di assi deve essere una stringa di tre caratteri "
                                    "tra 'X', 'Y' e 'Z' o 'x', 'y' e 'z'.");
    }
    return 1;
}

// Conversione da gradi a radianti
double Rotation::deg2rad(double degree)
{
    return degree * M_PI / 180.0;
}

// Conversione da radianti a gradi
double Rotation::rad2deg(double radian)
{
    return radian * 180.0 / M_PI;
}

Eigen::Quaterniond Rotation::quaternion_difference(const Eigen::Quaterniond &a,
                                                   const Eigen::Quaterniond &b)
{
    return a.inverse() * b;
}

Eigen::Quaterniond Rotation::quaternion_negation(const Eigen::Quaterniond &v)
{
    return Eigen::Quaterniond(-v.w(), -v.x(), -v.y(), -v.z());
}

Eigen::Quaterniond Rotation::scalar_product(const Eigen::Quaterniond &v, double t)
{
    return Eigen::Quaterniond(t * v.w(), t * v.x(), t * v.y(), t * v.z());
}

Eigen::Quaterniond Rotation::quaternion_minus(const Eigen::Quaterniond &v1,
                                              const Eigen::Quaterniond &v0)
{
    return Eigen::Quaterniond(v1.w() - v0.w(), v1.x() - v0.x(), v1.y() - v0.y(), v1.z() - v0.z());
}

Eigen::Quaterniond Rotation::quaternion_plus(const Eigen::Quaterniond &v1,
                                             const Eigen::Quaterniond &v0)
{
    return Eigen::Quaterniond(v1.w() + v0.w(), v1.x() + v0.x(), v1.y() + v0.y(), v1.z() + v0.z());
}

// Matrici di rotazione elementari
Eigen::Matrix3d Rotation::rotationX(double angle)
{
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot << 1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle);
    return rot;
}

Eigen::Matrix3d Rotation::rotationY(double angle)
{
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle);
    return rot;
}

Eigen::Matrix3d Rotation::rotationZ(double angle)
{
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1;
    return rot;
}

// Matrice S
Eigen::Matrix3d Rotation::matrixS(const Eigen::Vector3d &omega)
{
    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
    S << 0, -omega.z(), omega.y(), omega.z(), 0, -omega.x(), -omega.y(), omega.x(), 0;
    return S;
}

// Matrice R_dot
Eigen::Matrix3d Rotation::matrixR_dot(const Eigen::Matrix3d &R, const Eigen::Matrix3d &S)
{
    return S * R;
}

Eigen::Matrix3d Rotation::matrixR_dot(const Eigen::Matrix3d &R, const Eigen::Vector3d &omega)
{
    Eigen::Matrix3d S = matrixS(omega);
    return matrixR_dot(R, S);
}

Eigen::Matrix3d Rotation::matrixT(const Eigen::Vector3d &angles, const std::string &sequence)
{
    checkSequence(sequence);

    Eigen::Matrix3d T = Eigen::Matrix3d::Zero();

    Eigen::Vector3d theta[3];

    for (int i = 0; i < 3; ++i)
    {
        char c = sequence[std::string::size_type(i)];
        switch (c)
        {
        case 'X':
        case 'x':
            theta[i] = Eigen::Vector3d::UnitX();
            break;
        case 'Y':
        case 'y':
            theta[i] = Eigen::Vector3d::UnitY();
            break;
        case 'Z':
        case 'z':
            theta[i] = Eigen::Vector3d::UnitZ();
            break;
        default:
            break;
        }
    }

    T.col(0) = theta[0];

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    for (int i = 0; i < 2; ++i)
    {
        Eigen::Matrix3d Ri;

        switch (sequence[std::string::size_type(i)])
        {
        case 'X':
        case 'x':
            Ri = Rotation::rotationX(angles[i]);
            break;
        case 'Y':
        case 'y':
            Ri = Rotation::rotationY(angles[i]);
            break;
        case 'Z':
        case 'z':
            Ri = Rotation::rotationZ(angles[i]);
            break;
        default:
            Ri = Eigen::Matrix3d::Identity();
            break;
        }

        R *= Ri;
        T.col(i + 1) = R * theta[i + 1];
    }

    // Check if it is singular and so the determinant is near to zero
    if (std::abs(T.determinant()) < 1e-6)
    {
        std::cerr << "WARNING: The matrix is singular" << std::endl;
    }

    return T;
}

} // namespace MoveG
