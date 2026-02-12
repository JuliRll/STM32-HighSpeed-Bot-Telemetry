# STM32-HighSpeed-Bot-Telemetry

Este proyecto consiste en el desarrollo de un auto velocista autónomo basado en el microcontrolador STM32F103, capaz de alcanzar velocidades >2 m/s mediante el seguimiento de líneas con lógica PID. Incluye una estación de control remota desarrollada en Qt para la monitorización de datos en tiempo real.

Hardware & Tecnologías MCU: STM32F103 (ARM Cortex-M3) - Gestión de control en tiempo real.
  Sensores: Arreglo de infrarrojos para detección de línea con procesamiento vía ADC/DMA.
  Comunicación: Módulo WiFi para telemetría inalámbrica.
  Firmware: Desarrollado en C (STM32CubeIDE).
  Software de Control: Aplicación de escritorio multiplataforma desarrollada con Qt Framework (C++).

Funcionalidades Clave
  Lazo de Control PID: Optimizado para alta frecuencia de muestreo, permitiendo navegación estable a máxima velocidad.
  Telemetría On-the-fly: La interfaz en Qt permite ajustar las constantes K_p, K_i y K_d en tiempo real sin necesidad de reprogramar el firmware.
  Visualización de Datos: Gráficos de error y velocidad para análisis de performance en pista.
