#ifndef INTERRUPT_PRIORITIES_H
#define INTERRUPT_PRIORITIES_H

#if 0 // original
#define IRQ_PRIORITY_DMA_SPI_RX 0
#define IRQ_PRIORITY_TIM1       1
#define IRQ_PRIORITY_TIM8       1
#define IRQ_PRIORITY_DMA_SPI_TX 2
#define IRQ_PRIORITY_ADC        2
#define IRQ_PRIORITY_SPI        3

#else // experiment

#define IRQ_PRIORITY_DMA_SPI_RX 0
#define IRQ_PRIORITY_DMA_SPI_TX 0
#define IRQ_PRIORITY_SPI        0  // Same as DMA - callback completes atomically
#define IRQ_PRIORITY_TIM1       1  // Lower than SPI - can't interrupt callback
#define IRQ_PRIORITY_TIM8       1
#define IRQ_PRIORITY_ADC        2
#endif

#endif
