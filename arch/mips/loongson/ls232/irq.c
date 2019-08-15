/*
 * Copyright (C) 2000 RidgeRun, Inc.
 * Author: RidgeRun, Inc.
 *   glonnon@ridgerun.com, skranz@ridgerun.com, stevej@ridgerun.com
 *
 * Copyright 2001 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 * Copyright (C) 2000, 2001 Ralf Baechle (ralf@gnu.org)
 * Copyright (C) 2003 ICT CAS (guoyi@ict.ac.cn)
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel_stat.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/irq.h>
#include <linux/ptrace.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/irq_cpu.h>
#include <asm/i8259.h>
#include <asm/mipsregs.h>
#include <asm/system.h>
#include <ls1x_board.h>
#include <ls1x_board_int.h>
#include <ls1x_board_dbg.h>

#define LS232_CP0_CAUSE_TI  (1UL    <<  30)
#define LS232_CP0_CAUSE_PCI (1UL    <<  26)
#define LS232_COUNTER1_OVERFLOW     (1ULL    << 31)
#define LS232_COUNTER2_OVERFLOW     (1ULL   << 31)

/* Basic Reg Read/Write */
#define                             WriteReg(BaseAddress, RegOffset, Data) \
                                    MyOut32((BaseAddress) + (RegOffset), (unsigned int)(Data))

#define                             ReadReg(BaseAddress, RegOffset) \
	                                MyIn32((BaseAddress) + (RegOffset))

static void MyOut32(unsigned int Addr, unsigned int Value);
static unsigned int MyIn32(unsigned int Addr);

#define XIN_BASE_ADDR				0xbfd19000
#define XIN_ISR_OFFSET      		0x0	/* Interrupt Status Register */
#define XIN_IPR_OFFSET      		0x4	/* Interrupt Pending Register */
#define XIN_IER_OFFSET      		0x8	/* Interrupt Enable Register */
#define XIN_IAR_OFFSET      		0xC	/* Interrupt Acknowledge Register */
#define XIN_SIE_OFFSET      		0x10	/* Set Interrupt Enable Register */
#define XIN_CIE_OFFSET      		0x14	/* Clear Interrupt Enable Register */
#define XIN_IVR_OFFSET      		0x18	/* Interrupt Vector Register */
#define XIN_MER_OFFSET      		0x1C	/* Master Enable Register */
#define XIN_IMR_OFFSET      		0x20	/* Interrupt Mode Register , this is present
				 *  only for Fast Interrupt */
#define XIN_ILR_OFFSET      		0X24  /* Interrupt level register */
#define XIN_IVAR_OFFSET     		0x100 /* Interrupt Vector Address Register
				   * Interrupt 0 Offest, this is present
				   * only for Fast Interrupt */
#define XIN_IVEAR_OFFSET     		0x200 /* Interrupt Vector Extended Address
					Register */

#define XIN_MASK_UART0_ACK			0x1
#define XIN_MASK_UARTLITE_ACK		0x2

#define HUART_BASE_ADDR             0xbfd18000
#define HUART_RX_FIFO_OFFSET	    0x0	        /* receive FIFO, read only */
#define HUART_TX_FIFO_OFFSET	    0x4	        /* transmit FIFO, write only */
#define HUART_STATUS_REG_OFFSET		0x8	        /* status register, read only */
#define HUART_CONTROL_REG_OFFSET    0xc	        /* control reg, write only */

/* Control Register bit positions */

#define HUART_CR_ENABLE_INTR	    0x10	    /* enable interrupt */
#define HUART_CR_FIFO_RX_RESET		0x02	    /* reset receive FIFO */
#define HUART_CR_FIFO_TX_RESET		0x01	    /* reset transmit FIFO */

/* Status Register bit positions */

#define HUART_SR_PARITY_ERROR       0x80
#define HUART_SR_FRAMING_ERROR		0x40
#define HUART_SR_OVERRUN_ERROR		0x20
#define HUART_SR_INTR_ENABLED	    0x10	    /* interrupt enabled */
#define HUART_SR_TX_FIFO_FULL	    0x08	    /* transmit FIFO full */
#define HUART_SR_TX_FIFO_EMPTY		0x04	    /* transmit FIFO empty */
#define HUART_SR_RX_FIFO_FULL	    0x02	    /* receive FIFO full */
#define HUART_SR_RX_FIFO_VALID_DATA	0x01	    /* data in receive FIFO */

static struct ls1x_board_intc_regs volatile *ls1x_board_hw0_icregs
	= (struct ls1x_board_intc_regs volatile *)(KSEG1ADDR(LS1X_BOARD_INTREG_BASE));

void ls1x_board_hw_irqdispatch(int n);
void ls1x_board_irq_init();

asmlinkage void plat_irq_dispatch(struct pt_regs *regs)
{
	// unsigned int pending = read_c0_cause() & read_c0_status() & ST0_IM;
	unsigned int cause_reg = read_c0_cause() ;
	unsigned int status_reg = read_c0_status() ;
	unsigned volatile int cause = cause_reg & ST0_IM;
	unsigned volatile int status = status_reg & ST0_IM;
	unsigned int pending = cause & status;
	unsigned int Register = 0;
	uint64_t volatile counter1, counter2;
   	 if (pending & CAUSEF_IP7) {
#if 0
        	counter1 = read_c0_perfcntr0();
	        counter2 = read_c0_perfcntr1();
	       if (((counter1 & LS232_COUNTER1_OVERFLOW)||(counter2 & LS232_COUNTER2_OVERFLOW)) && (cause_reg & LS232_CP0_CAUSE_PCI))
        	{
               		 do_IRQ(MIPS_CPU_IRQ_BASE+6);
        	}
	        if (cause_reg & LS232_CP0_CAUSE_TI)
        	{
	                 do_IRQ(MIPS_CPU_IRQ_BASE+7);
	
        	}
#else
	                 do_IRQ(MIPS_CPU_IRQ_BASE+7);
#endif
        }
    else if (pending & CAUSEF_IP2) {
	    do_IRQ(MIPS_CPU_IRQ_BASE+2);
	}
    else if (pending & CAUSEF_IP3) 
	{
		// Read INTC Status
		Register = ReadReg(XIN_BASE_ADDR, XIN_ISR_OFFSET);

			// If UART0, proceed it normally
		if(Register && XIN_MASK_UART0_ACK == XIN_MASK_UART0_ACK)
		{
	    	do_IRQ(MIPS_CPU_IRQ_BASE+3);

			// Check if UART Lite INT is pending
			Register = ReadReg(XIN_BASE_ADDR, XIN_IPR_OFFSET);
			if(Register && XIN_MASK_UART0_ACK == XIN_MASK_UART0_ACK)
			{
				Register = ReadReg(HUART_BASE_ADDR, HUART_STATUS_REG_OFFSET);
				if(Register & HUART_SR_TX_FIFO_EMPTY != HUART_SR_TX_FIFO_EMPTY)
					printk("XLNX UART LITE Required for INT.\n");
			}
		}
			// else redirect to our UART_Lite
		else if(Register && XIN_MASK_UARTLITE_ACK == XIN_MASK_UARTLITE_ACK)
		{
			if(Register & HUART_SR_TX_FIFO_EMPTY != HUART_SR_TX_FIFO_EMPTY)
				printk("XLNX UART LITE Required for INT.\n");
		}
		else
		{
			// printk("INTC Error!\n");
			// Set ACK and do normal UART0 IRQ
			do_IRQ(MIPS_CPU_IRQ_BASE+3);
			Register = ReadReg(XIN_BASE_ADDR, XIN_ISR_OFFSET);
			WriteReg(XIN_BASE_ADDR, XIN_IAR_OFFSET, 0x3);
		}

		// Set ACK
		Register = ReadReg(XIN_BASE_ADDR, XIN_ISR_OFFSET);
		WriteReg(XIN_BASE_ADDR, XIN_IAR_OFFSET, Register | 0x3);
	}
    else if (pending & CAUSEF_IP4) {
	    do_IRQ(MIPS_CPU_IRQ_BASE+4);
	}
    else if (pending & CAUSEF_IP5) {
	    do_IRQ(MIPS_CPU_IRQ_BASE+5);
	}
    else if (pending & CAUSEF_IP6) {
	    do_IRQ(MIPS_CPU_IRQ_BASE+6);
    } else {
        spurious_interrupt();
    }

}

static struct irqaction cascade_irqaction = {
	.handler	= no_action,
//        .mask		= CPU_MASK_NONE,
	.name		= "cascade",
};

void __init arch_init_irq(void)
{
	int i;
    unsigned long flags;

	clear_c0_status(ST0_IM | ST0_BEV);
	local_irq_disable();


	mips_cpu_irq_init();	

	printk("HITwh NSCSCC Team: Initializing Interrupt Controller...\n");
	// m1 0xbfd19008 0x00000001 - IER
	WriteReg(XIN_BASE_ADDR, XIN_IER_OFFSET, 0x3);
	printk("HITwh NSCSCC Team: Enabled INTC interrupt set[0x%08x].\n", ReadReg(XIN_BASE_ADDR, XIN_IER_OFFSET));
	// m1 0xbfd1901C 0x00000003 - MER
	WriteReg(XIN_BASE_ADDR, XIN_MER_OFFSET, 0x3);
	printk("HITwh NSCSCC Team: Enabled INTC Hardware interrupt and global int[0x%08x].\n", ReadReg(XIN_BASE_ADDR, XIN_MER_OFFSET));

	// UART LITE init
	printk("HITwh NSCSCC Team: Initializing XUart Lite...\n");
    // Diable all the Interrupt
    WriteReg(HUART_BASE_ADDR, HUART_CONTROL_REG_OFFSET, 0);

    printk("HITwh NSCSCC Team: Running UART Lite Self-Check...");

    // Read Status
    unsigned int RegData = ReadReg(HUART_BASE_ADDR, HUART_STATUS_REG_OFFSET);
    // Maintain INTE
    RegData &= HUART_SR_INTR_ENABLED;

    // Reset FIFOs
    WriteReg(HUART_BASE_ADDR, HUART_CONTROL_REG_OFFSET,
			RegData | HUART_CR_FIFO_TX_RESET | HUART_CR_FIFO_RX_RESET);

    // Read again
    RegData = ReadReg(HUART_BASE_ADDR, HUART_STATUS_REG_OFFSET);
	/*
	 * If the status register is any other value other than
	 * HUART_SR_TX_FIFO_EMPTY then the test is a failure since this is
	 * the not the value after reset
	 */
	if (RegData != HUART_SR_TX_FIFO_EMPTY) 
    {
        printk("Failed!.\n");
        printk("HITwh NSCSCC Team: Reg values differs from expected values!.\n");
        printk("HITwh NSCSCC Team: Preffering = 0x00000004 | Current = 0x%08x.\n", RegData);
		return;
	}

    printk("Succeed!.\n");

    // Enable INT
    printk("HITwh NSCSCC Team: Enable interrupt......");
    RegData = ReadReg(HUART_BASE_ADDR, HUART_STATUS_REG_OFFSET);
    RegData |= HUART_SR_INTR_ENABLED;
    WriteReg(HUART_BASE_ADDR, HUART_CONTROL_REG_OFFSET, RegData);
    RegData = ReadReg(HUART_BASE_ADDR, HUART_STATUS_REG_OFFSET);
    printk("Done.[0x%08x]\n", RegData);
}


static void MyOut32(unsigned int Addr, unsigned int Value)
{
	volatile unsigned int *LocalAddr = (volatile unsigned int *)Addr;
	*LocalAddr = Value;
}

static unsigned int MyIn32(unsigned int Addr)
{
	return *(volatile unsigned int *) Addr;
}
