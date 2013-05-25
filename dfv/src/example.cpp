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
    // Ejemplos de operaciones con Vector3
    std::cout << "===========================================" << std::endl;
    std::cout << "Programa de demostración de la librería dfv" << std::endl;
    std::cout << "===========================================" << std::endl;
    
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
    
    return 0;
}
