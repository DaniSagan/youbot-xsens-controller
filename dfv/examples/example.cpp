/*
 * Pequeño ejemplo de como usar la librería dfv
 *
 * Autor: Daniel Fernández Villanueva
 * Mayo de 2013
 */

#include <iostream>
#include <dfv/dfv.h>

int main(int argc, char** argv)
{
    // ***************************************************** //
    // ******** Ejemplos de operaciones con Vector3 ******** //
    // ***************************************************** //
    std::cout << "===========================================" << std::endl;
    std::cout << "Programa de demostración de la librería dfv" << std::endl;
    std::cout << "===========================================" << std::endl;
    
    std::cout << std::endl;
    std::cout << "Clase Vector3:" << std::endl;
    std::cout << "--------------" << std::endl;
    
    dfv::Vector3 v1(1.0, 2.0, 3.0);
    dfv::Vector3 v2(4.0, 5.0, 6.0);
    
    std::cout << "v1 = " << v1 << std::endl;
    std::cout << "v2 = " << v2 << std::endl;
    
    dfv::Vector3 v3;
    
    std::cout << "Suma de vectores: " << std::endl;
    v3 = v1 + v2;
    std::cout << "\tv1 + v2 = " << v3 << std::endl;
    
    std::cout << "Resta de vectores: " << std::endl;
    v3 = v1 - v2;
    std::cout << "\tv1 - v2 = " << v3 << std::endl;
    
    std::cout << "Producto por un escalar: " << std::endl;
    v3 = 2.0 * v1;
    std::cout << "\t2.0*v1 = " << v3 << std::endl;
    
    std::cout << "Producto escalar: " << std::endl;
    double p = v1 * v2;
    std::cout << "\tv1 · v2 = " << p << std::endl;
    
    std::cout << "Producto vectorial: " << std::endl;
    v3 = v1 ^ v2;
    std::cout << "\tv1 ^ v2 = " << v3 << std::endl;
    
    std::cout << "Magnitud: " << std::endl;
    double m = v1.GetMagnitude();
    std::cout << "\t||v1|| = " << m << std::endl;
    
    std::cout << "Vector normalizado: " << std::endl;
    v3 = v1.GetNormalized();
    std::cout << "\tv1_u = " << v3 << std::endl;
    
    std::cout << "Versores: " << std::endl;
    std::cout << "\ti = " << dfv::Vector3::i << std::endl;
    std::cout << "\tj = " << dfv::Vector3::j << std::endl;
    std::cout << "\tk = " << dfv::Vector3::k << std::endl;
    
    // ******************************************************** //
    // ******** Ejemplos de operaciones con Quaternion ******** //
    // ******************************************************** //
    
    std::cout << std::endl;
    std::cout << "Clase Quaternion:" << std::endl;
    std::cout << "-----------------" << std::endl;
    
    dfv::Quaternion q1(1.0, 2.0, 3.0, 4.0);
    dfv::Quaternion q2(5.0, 6.0, 7.0, 8.0);
    
    std::cout << "q1 = " << q1 << std::endl;
    std::cout << "q2 = " << q2 << std::endl;
    
    dfv::Quaternion q3;
    
    std::cout << "Suma de cuaterniones: " << std::endl;
    q3 = q1 + q2;
    std::cout << "\tq1 + q2 = " << q3 << std::endl;
    
    std::cout << "Resta de cuaterniones: " << std::endl;
    q3 = q1 - q2;
    std::cout << "\tq1 - q2 = " << q3 << std::endl;
    
    std::cout << "Producto por un escalar: " << std::endl;
    q3 = 2.0 * q1;
    std::cout << "\t2.0*q1 = " << q3 << std::endl;
    
    std::cout << "Normalización: " << std::endl;
    q1.Normalize();
    q2.Normalize();
    std::cout << "\tq1 := " << q1 << std::endl;
    std::cout << "\tq2 := " << q2 << std::endl;
    
    std::cout << "Módulo: " << std::endl;
    m = q1.GetModulus();
    std::cout << "\t||q1|| = " << m << std::endl;
    
    std::cout << "Producto de Hamilton: " << std::endl;
    q3 = q1 * q2;
    std::cout << "\tq1 * q2 = " << q3 << std::endl;
    
    std::cout << "Rotación de un vector:" << std::endl;
    dfv::Vector3 v4 = dfv::Vector3::i;
    dfv::Vector3 axis = dfv::Vector3::j;
    double angle = dfv::DegToRad(90.0);
    dfv::Quaternion q_rot = dfv::Quaternion::GetRotationQuaternion(axis, angle);
    std::cout << "\tv4 = " << v4 << std::endl;
    std::cout << "\teje = " << axis << std::endl;
    std::cout << "\tángulo = " << angle << " rad" << std::endl;
    std::cout << "\tq_rot = " << q_rot << std::endl;
    v4.Rotate(q_rot);
    std::cout << "\tv4 := " << v4 << std::endl;
    
    std::cout << "Obtención del eje y ángulo asociados al cuaternión: " << std::endl;
    std::cout << "\tq_rot = " << q_rot << std::endl;
    dfv::Vector3 axis_rot;
    double angle_rot;
    q_rot.GetAxisAndAngle(axis_rot, angle_rot);
    std::cout << "\teje: " << axis_rot << std::endl;
    std::cout << "\tángulo: " << angle_rot << " rad" << std::endl;
    
    // **************************************************** //
    // ******** Ejemplos de operaciones con Matrix ******** //
    // **************************************************** //
    
    std::cout << std::endl;
    std::cout << "Clase Matrix:" << std::endl;
    std::cout << "-------------" << std::endl;
    
    dfv::Matrix m1(3, 3);
    m1.Randomize();
    dfv::Matrix m2(3, 1);
    m2.Randomize();
    std::cout << "m1:" << std::endl;
    std::cout << m1 << std::endl;
    std::cout << "m2:" << std::endl;
    std::cout << m2 << std::endl;
    
    dfv::Matrix m3 = m1 * m2;
    std::cout << "m1 * m2:" << std::endl;
    std::cout << m3 << std::endl;
    
    dfv::Matrix m4 = m1.GetInverse();
    std::cout << "Inversa de m1:" << std::endl;
    std::cout << m4 << std::endl;
    
    double det = m1.GetDeterminant();
    std::cout << "Determinante de m1: " << det << std::endl;
    
    dfv::Matrix m5 = dfv::Matrix::Identity(5);
    std::cout << "Matriz unitaria de dimensión 5: " << std::endl;
    std::cout << m5 << std::endl;
    
    m5.Randomize();
    std::cout << "Matriz aleatoria dimensión 5: " << std::endl;
    std::cout << m5 << std::endl;
    
    dfv::Matrix m6 = m5.GetInverse();
    std::cout << "Inversa de la matriz: " << std::endl;
    std::cout << m6 << std::endl;
    
    // ************************************ //
    // ******** Ejemplos con utils ******** //
    // ************************************ //
    
    std::cout << std::endl;
    std::cout << "Utils:" << std::endl;
    std::cout << "------" << std::endl;
    
    std::string str = "Ejemplo de utilización de la función StrTokenize.";
    std::cout << str << std::endl << "Tokens: " << std::endl;
    std::vector<std::string> tokens = dfv::StrTokenize(str, ' ');
    for(std::vector<std::string>::iterator it = tokens.begin();
        it != tokens.end(); ++it)
    {
        std::cout << *it << std::endl;
    }
    
    // **************************************************** //
    // ******** Ejemplos de operaciones con Youbot ******** //
    // **************************************************** //
    
    ros::init(argc, argv, "dfv_example");
    ros::NodeHandle node_handle;
    dfv::Youbot youbot(node_handle); // creación de un objeto youbot
    
    
    return 0;
}
