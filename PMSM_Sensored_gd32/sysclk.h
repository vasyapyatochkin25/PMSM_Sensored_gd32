#include "gd32f10x_rcu.h"
void SetSysClockTo72(void)
{
	uint32_t timeout = 0U;
	uint32_t stab_flag = 0U;

	/* select IRC8M as system clock source, deinitialize the RCU */
	rcu_system_clock_source_config(RCU_CKSYSSRC_IRC8M);
	rcu_deinit();
    
	/* enable HXTAL */
	RCU_CTL |= RCU_CTL_HXTALEN;

	/* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
	do {
		timeout++;
		stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
	} while ((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

	/* if fail */
	if (0U == (RCU_CTL & RCU_CTL_HXTALSTB)) {
		while (1) {
		}
	}

	/* HXTAL is stable */
	/* AHB = SYSCLK */
	RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
	/* APB2 = AHB/1 */
	RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
	/* APB1 = AHB/2 */
	RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

#if (defined(GD32F10X_MD) || defined(GD32F10X_HD) || defined(GD32F10X_XD))
	/* select HXTAL/2 as clock source */
	RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0);
	RCU_CFG0 |= (RCU_PLLSRC_HXTAL | RCU_CFG0_PREDV0);

	/* CK_PLL = (CK_HXTAL/2) * 18 = 72 MHz */
	RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
	RCU_CFG0 |= RCU_PLL_MUL18;

#elif defined(GD32F10X_CL)
	/* CK_PLL = (CK_PREDIV0) * 9 = 36 MHz */
	RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
	RCU_CFG0 |= (RCU_PLLSRC_HXTAL | RCU_PLL_MUL9);

	/* CK_PREDIV0 = (CK_HXTAL)/5 *8 /10 = 4 MHz */
	RCU_CFG1 &= ~(RCU_CFG1_PREDV0SEL | RCU_CFG1_PLL1MF | RCU_CFG1_PREDV1 | RCU_CFG1_PREDV0);
	RCU_CFG1 |= (RCU_PREDV0SRC_CKPLL1 | RCU_PLL1_MUL8 | RCU_PREDV1_DIV5 | RCU_PREDV0_DIV10);

	/* enable PLL1 */
	RCU_CTL |= RCU_CTL_PLL1EN;
	/* wait till PLL1 is ready */
	while ((RCU_CTL & RCU_CTL_PLL1STB) == 0) {
	}
#endif /* GD32F10X_MD and GD32F10X_HD and GD32F10X_XD */

	/* enable PLL */
	RCU_CTL |= RCU_CTL_PLLEN;

	/* wait until PLL is stable */
	while (0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
	}

	/* select PLL as system clock */
	RCU_CFG0 &= ~RCU_CFG0_SCS;
	RCU_CFG0 |= RCU_CKSYSSRC_PLL;

	/* wait until PLL is selected as system clock */
	while (RCU_SCSS_PLL != (RCU_CFG0 & RCU_CFG0_SCSS)) {
	}
}

void switch_system_clock_to_108m_irc8m(void)
{
	uint32_t timeout = 0U;
	uint32_t stab_flag = 0U;
    
	/* select IRC8M as system clock source, deinitialize the RCU */
	rcu_system_clock_source_config(RCU_CKSYSSRC_IRC8M);
	rcu_deinit();
    
	/* enable IRC8M */
	RCU_CTL |= RCU_CTL_IRC8MEN;

	/* wait until IRC8M is stable or the startup time is longer than IRC8M_STARTUP_TIMEOUT */
	do {
		timeout++;
		stab_flag = (RCU_CTL & RCU_CTL_IRC8MSTB);
	} while ((0U == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

	/* if fail */
	if (0U == (RCU_CTL & RCU_CTL_IRC8MSTB)) {
		while (1) {
		}
	}

	/* IRC8M is stable */
	/* AHB = SYSCLK */
	RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
	/* APB2 = AHB/1 */
	RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
	/* APB1 = AHB/2 */
	RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

	/* CK_PLL = (CK_IRC8M/2) * 27 = 108 MHz */
	RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
	RCU_CFG0 |= RCU_PLL_MUL27;

	/* enable PLL */
	RCU_CTL |= RCU_CTL_PLLEN;

	/* wait until PLL is stable */
	while (0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
	}

	/* select PLL as system clock */
	RCU_CFG0 &= ~RCU_CFG0_SCS;
	RCU_CFG0 |= RCU_CKSYSSRC_PLL;

	/* wait until PLL is selected as system clock */
	while (RCU_SCSS_PLL != (RCU_CFG0 & RCU_CFG0_SCSS)) {
	}
}

void switch_system_clock_to_72m_irc8m(void)
{
	uint32_t timeout = 0U;
	uint32_t stab_flag = 0U;
    
	/* select IRC8M as system clock source, deinitialize the RCU */
	rcu_system_clock_source_config(RCU_CKSYSSRC_IRC8M);
	rcu_deinit();
    
	/* enable IRC8M */
	RCU_CTL |= RCU_CTL_IRC8MEN;

	/* wait until IRC8M is stable or the startup time is longer than IRC8M_STARTUP_TIMEOUT */
	do {
		timeout++;
		stab_flag = (RCU_CTL & RCU_CTL_IRC8MSTB);
	} while ((0U == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

	/* if fail */
	if (0U == (RCU_CTL & RCU_CTL_IRC8MSTB)) {
		while (1) {
		}
	}

	/* IRC8M is stable */
	/* AHB = SYSCLK */
	RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
	/* APB2 = AHB/1 */
	RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
	/* APB1 = AHB/2 */
	RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

	/* CK_PLL = (CK_IRC8M/2) * 27 = 108 MHz */
	RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
	RCU_CFG0 |= RCU_PLL_MUL18;//72MHz

	/* enable PLL */
	RCU_CTL |= RCU_CTL_PLLEN;

	/* wait until PLL is stable */
	while (0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
	}

	/* select PLL as system clock */
	RCU_CFG0 &= ~RCU_CFG0_SCS;
	RCU_CFG0 |= RCU_CKSYSSRC_PLL;

	/* wait until PLL is selected as system clock */
	while (RCU_SCSS_PLL != (RCU_CFG0 & RCU_CFG0_SCSS)) {
	}
}

void delay_1ms(uint32_t count)
{
	uint32_t delay = count;

	while (0U != delay) {
		delay--;
	}
}