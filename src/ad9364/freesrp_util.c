#include "freesrp_util.h"

#include "ad9364/ad9361_api.h"
#include "ad9364/parameters.h"

void print_ensm_state(struct ad9361_rf_phy *phy)
{
	static uint32_t mode;

	ad9361_get_en_state_machine_mode(phy, &mode);

	switch(mode)
	{
	case ENSM_MODE_TX:
		printf("INFO: AD9364 in TX mode\n\r");
		break;
	case ENSM_MODE_RX:
		printf("INFO: AD9364 in RX mode\n\r");
		break;
	case ENSM_MODE_ALERT:
		printf("INFO: AD9364 in ALERT mode\n\r");
		break;
	case ENSM_MODE_FDD:
		printf("INFO: AD9364 in FDD mode\n\r");
		break;
	case ENSM_MODE_WAIT:
		printf("INFO: AD9364 in WAIT mode\n\r");
		break;
	case ENSM_MODE_SLEEP:
		printf("INFO: AD9364 in SLEEP mode\n\r");
		break;
	case ENSM_MODE_PINCTRL:
		printf("INFO: AD9364 in PINCTRL mode\n\r");
		break;
	case ENSM_MODE_PINCTRL_FDD_INDEP:
		printf("INFO: AD9364 in PINCTRL_FDD_INDEP mode\n\r");
		break;
	}
}

void ctrl_set_value(uint8_t gpio, uint8_t value)
{
	/* TODO: CTRL GPIO for band selection
	static u32 state = 0;
	//state = XGpio_DiscreteRead(&ctrl_gpio, 1);
	if(value)
	{
		state |= (1 << gpio);
	}
	else
	{
		state &= ~(1 << gpio);
	}
	XGpio_DiscreteWrite(&ctrl_gpio, 1, state);
	*/
}

void rx_band_select(uint32_t port)
{
	switch(port)
	{
	case A_BALANCED:
		ctrl_set_value(GPIO_BANDSEL_RX_B, 0);
		ctrl_set_value(GPIO_BANDSEL_RX_C, 0);
		ctrl_set_value(GPIO_BANDSEL_RX_A, 1);
		break;
	case B_BALANCED:
		ctrl_set_value(GPIO_BANDSEL_RX_A, 0);
		ctrl_set_value(GPIO_BANDSEL_RX_C, 0);
		ctrl_set_value(GPIO_BANDSEL_RX_B, 1);
		break;
	case C_BALANCED:
		ctrl_set_value(GPIO_BANDSEL_RX_A, 0);
		ctrl_set_value(GPIO_BANDSEL_RX_B, 0);
		ctrl_set_value(GPIO_BANDSEL_RX_C, 1);
		break;
	}
}

void tx_band_select(uint32_t port)
{
	switch(port)
	{
	case TXA:
		ctrl_set_value(GPIO_BANDSEL_TX_B, 0);
		ctrl_set_value(GPIO_BANDSEL_TX_A, 1);
		break;
	case TXB:
		ctrl_set_value(GPIO_BANDSEL_TX_A, 0);
		ctrl_set_value(GPIO_BANDSEL_TX_B, 1);
		break;
	}
}
