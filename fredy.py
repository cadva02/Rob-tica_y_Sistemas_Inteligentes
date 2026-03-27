#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Módulo Fredy - Pruebas y ejercicios
"""

class Fredy:
    """Clase Fredy para realizar operaciones"""
    
    def __init__(self, nombre="Fredy"):
        self.nombre = nombre
    
    def saludar(self):
        """Saluda al usuario"""
        return f"¡Hola! Soy {self.nombre}"
    
    def sumar(self, a, b):
        """Suma dos números"""
        return a + b
    
    def multiplicar(self, a, b):
        """Multiplica dos números"""
        return a * b


def main():
    """Función principal"""
    fredy = Fredy("Fredy")
    print(fredy.saludar())
    print(f"2 + 3 = {fredy.sumar(2, 3)}")
    print(f"4 × 5 = {fredy.multiplicar(4, 5)}")


if __name__ == "__main__":
    main()
