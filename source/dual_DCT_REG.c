/****************************************************************
* FILENAME:     dual_DCT_REG.c
* DESCRIPTION:  dual DCT controller (regulator) which is reducing periodic disturbance
* AUTHOR:       Denis Su�in
* START DATE:   28.8.2019
* VERSION:      3.1
*
* CHANGES :
* VERSION   	DATE		WHO					DETAIL
* 1.0       	28.8.2019	Denis Su�in			Initial version based on DCT controller v3.4.
* 2.0       	20.9.2019	Denis Su�in			Upgraded phase compensation, now is phase 
*												compensation accepted in degrees
* 2.1       	7.11.2019	Denis Su�in			Sign "+" changed with "-" for stable operation,
*												because of FIR filter library
* 3.0       	3.12.2019	Denis Su�in			Coefficient recalculation updated without using
* 												for loop
* 3.1       	19.3.2020	Denis Su�in			Amplitude implementation on both DCT filters
*
****************************************************************/

#include "dual_DCT_REG.h"

// deklaracija funkcij

// globalne spremenljivke




/****************************************************************************************************
* Funkcija, ki izvede algoritem dvojnega DCT regulatorja.
* Zelo za�eleno je, da je razmerje med vzor�no frekvenco in osnovno frekvenco reguliranega signala
* enako velikosti pomnilnika "BufferHistoryLength" (in ve�je od 20), saj je regulator na to ob�utljiv,
* kar lahko privede do nezanemarljivega pogre�ka v stacionarnem stanju.
****************************************************************************************************/
#pragma CODE_SECTION(dual_DCT_REG_CALC, "ramfuncs");
void dual_DCT_REG_CALC (dual_DCT_REG_float *v)
{
	// lokalne spremenljivke




	// program

	// omejitev dol�ine circular bufferja
	if (v->BufferHistoryLength > FIR_FILTER_NUMBER_OF_COEFF2)
	{
		v->BufferHistoryLength = FIR_FILTER_NUMBER_OF_COEFF2;
	}
	else if (v->BufferHistoryLength < 0)
	{
		v->BufferHistoryLength = 0;
	}

	// to�no dolo�ena velikost kro�nega pomnilnika in �t. koeficientov DCT (FIR) filtra
	v->BufferHistoryLength = FIR_FILTER_NUMBER_OF_COEFF2;




	// omejitev vzor�nega signala med 0.0 in 0.9999
	//(SamplingSignal ne sme biti enak ena, ker mora biti indeks i omejen od 0 do BufferHistoryLength-1)
	v->SamplingSignal = (v->SamplingSignal > 0.99999)? 0.99999: v->SamplingSignal;
	v->SamplingSignal = (v->SamplingSignal < 0.0)? 0.0: v->SamplingSignal;




	// izra�un vsote bufferja, kjer so dolo�eni harmoniki, ki jih reguliramo


	// preveri, �e je uporabnik spremenil vrednost harmonika, njegovo amplitudo ali kompenzacijo faze
	v->SumOfHarmonics = v->SumOfHarmonics + v->HarmonicsBuffer[v->HarmonicIndex];
	v->SumOfAmplitudes1 = v->SumOfAmplitudes1 + v->A1[v->HarmonicIndex];
	v->SumOfAmplitudes2 = v->SumOfAmplitudes2 + v->A2[v->HarmonicIndex];
	v->SumOfLagCompensation = v->SumOfLagCompensation + v->fi_deg[v->HarmonicIndex];

	// pove�aj indeks zaporednega harmonika
	v->HarmonicIndex = v->HarmonicIndex + 1;

	// �e indeks preraste obseg harmonikov,
	if(v->HarmonicIndex >= LENGTH_OF_HARMONICS_ARRAY2)
	{
		// zaznaj spremembe parametrov dvojnega DCT regulatorja s strani uporabnika, ki zahteva vnovi�ni izra�un koeficientov obeh DCT filtrov
		if( (v->SumOfHarmonics != v->SumOfHarmonicsOld) || (v->SumOfAmplitudes1 != v->SumOfAmplitudesOld1) || (v->SumOfAmplitudes2 != v->SumOfAmplitudesOld2) || (v->SumOfLagCompensation != v->SumOfLagCompensationOld) )
		{
			// postavi zastavico in za�ni izra�un koeficientov
			v->CoeffCalcInProgressFlag = 1;
		}

		// resetiraj indeks zaporednega harmonika
		v->HarmonicIndex = 0;

		// shranim vrednosti vektorjev (array-ev), ki bodo v naslednjem ciklu prej�nje vrednosti
		v->SumOfHarmonicsOld = v->SumOfHarmonics;
		v->SumOfAmplitudesOld1 = v->SumOfAmplitudes1;
		v->SumOfAmplitudesOld2 = v->SumOfAmplitudes2;
		v->SumOfLagCompensationOld = v->SumOfLagCompensation;

		// po�isti akumulirane dele
		v->SumOfHarmonics = 0;
		v->SumOfAmplitudes1 = 0.0;
		v->SumOfAmplitudes2 = 0.0;
		v->SumOfLagCompensation = 0;
	}

	// izra�un koeficientov DCT filtra 1 in 2; vsako vzor�no periodo en koeficient (s prispevkom le enega harmonika)
	if(v->CoeffCalcInProgressFlag != 0)
	{
		// calculate new coefficients
		FIR_FILTER_COEFF_CALC2(v);
	}




    // izra�un trenutnega indeksa bufferja
	v->i = (int)(v->SamplingSignal*v->BufferHistoryLength);




	// �e se indeks spremeni, potem gre algoritem dalje:
	//   1. �e je "SamplingSignal" zunaj te funkcije natan�no sinhroniziran z vzor�no frekvenco (|i_delta| = 1)
	//      se algoritem izvajanja repetitivnega regulatorja izvede vsako vzor�no periodo/interval in se
	//      izkoristi celotna velikost pomnilnika, kar je optimalno
	//   2. �e je "SamplingSignal" prepo�asen (|i_delta| < 1) oz. osnovna frekvenca reguliranega signala prenizka,
	//      ni nujno, da se algoritem izvajanja repetitivnega regulatorja izvede vsako vzor�no periodo/interval,
	//      kar pomeni, da ta algoritem lahko deluje s frekvenco ni�jo od vzor�ne frekvence
	//   3. �e je "SamplingSignal" prehiter (|i_delta| > 1), oz. osnovna frekvenca reguliranega signala previsoka,
	//      se algoritem izvajanja repetitivnega regulatorja izvede vsako vzor�no periodo/interval,
	//      a se velikost pomnilnika umetno zmanj�a za faktor "i_delta", saj zapisujemo in beremo le vsak "i_delta"-ti vzorec,
	//      kar pomeni, kot da bi bila velikost pomnilnika manj�a za faktor "i_delta"
	//      OPOMBA: v->i_delta = v->i - v->i_prev!
	if ((v->i != v->i_prev))
	{




		/***************************************************/
		/* koda dvojnega DCT regulatorja */
		/***************************************************/

		// izra�unam trenutni error
		v->Err = v->Ref - v->Fdb;

		// izra�unam novi akumuliran error
		v->ErrSum = v->Kdct * v->Err +						\
					v->FIR_filter_float1.output;


		// omejim trenutni error, da ne gre v nasi�enje
		v->ErrSum = (v->ErrSum > v->ErrSumMax)? v->ErrSumMax: v->ErrSum;
		v->ErrSum = (v->ErrSum < v->ErrSumMin)? v->ErrSumMin: v->ErrSum;




		/* DCT filter 1 - without output compensation */
		v->FIR_filter_float1.input = v->ErrSum;
		v->FIR_filter_float1.calc(&(v->FIR_filter_float1));

		/* DCT filter 2 - with output compensation */
		v->FIR_filter_float2.input = v->ErrSum;
		v->FIR_filter_float2.calc(&(v->FIR_filter_float2));
		v->Correction = v->FIR_filter_float2.output;

		// izra�unam izhod
		v->Out = v->Correction;

	    // shranim vrednost indeksa i, ki bo v naslednjem ciklu prej�nji i
	    v->i_prev = v->i;

    } // end of if (i != i_prev)




    // omejim izhod
    v->Out = (v->Out > v->OutMax)? v->OutMax: v->Out;
    v->Out = (v->Out < v->OutMin)? v->OutMin: v->Out;




} // konec funkcije








/****************************************************************************************************
 * Izra�un koeficientov DCT (FIR) filtra 1 in 2 med delovanjem. V eni vzor�ni periodi je izra�unan
 * le en koeficient za le en posamezni harmonik.
 * OPOMBA: Za izra�un vseh N koeficientov, je potrebnih N vzor�nih period.
 * OPOMBA: FIR filter iz FPU knji�nice izvede konvolucijo le delno, ker ne obrne signala, zato ima
 *         kompenzacija zakasnitve ravno nasproten predznak.
****************************************************************************************************/
#pragma CODE_SECTION(FIR_FILTER_COEFF_CALC2, "ramfuncs");
void FIR_FILTER_COEFF_CALC2 (dual_DCT_REG_float *v)
{
	static int harmonic_index = 0;
	static int cleared_coeff_flag = 0;


	// before calculating the first harmonic contribution, clear the coefficient first
	if(cleared_coeff_flag == 0)
	{
		*(v->FIR_filter_float1.coeff_ptr + v->j) = 0.0;
		*(v->FIR_filter_float2.coeff_ptr + v->j) = 0.0;
	}

	// add the contribution of specific harmonic to the previous coefficent value

	/* LAG COMPENSATION HAS NEGATIVE SIGN, BECAUSE OF REALIZATION OF DCT FILTER WITH FPU LIBRARY */
	if(v->HarmonicsBuffer[harmonic_index] != 0)
	{
		*(v->FIR_filter_float1.coeff_ptr + v->j) = *(v->FIR_filter_float1.coeff_ptr + v->j) + 		\
				2.0/(FIR_FILTER_NUMBER_OF_COEFF2) *  												\
				v->A1[harmonic_index] * cos( 2.0 * PI * v->HarmonicsBuffer[harmonic_index] * 		\
						( (float)(v->j) ) / (FIR_FILTER_NUMBER_OF_COEFF2) );

		*(v->FIR_filter_float2.coeff_ptr + v->j) = *(v->FIR_filter_float2.coeff_ptr + v->j) + 		\
				2.0/(FIR_FILTER_NUMBER_OF_COEFF2) *  												\
				v->A2[harmonic_index] * cos( 2.0 * PI * v->HarmonicsBuffer[harmonic_index] * 		\
						( (float)(v->j) / (FIR_FILTER_NUMBER_OF_COEFF2) ) - 						\
						v->fi_deg[harmonic_index] * PI/180.0);										\
	}
	/* FIR FILTER FROM FPU LIBRARY DOESN'T FLIP SIGNAL FROM LEFT TO RIGHT WHILE PERFORMING CONVOLUTION */

	// increment coefficient index
	v->j = v->j + 1;

	// if the last coefficent is calculated,
	if(v->j >= FIR_FILTER_NUMBER_OF_COEFF2)
	{
		// reset coefficient index
		v->j = 0;

		// increment harmonic index
		harmonic_index = harmonic_index + 1;

		// signalize that all the coefficients were already cleared,
		// when calculating the first harmonic contribution
		cleared_coeff_flag = 1;
	}

	// if all the harmonics have been taken into account, stop the calculation
	if(harmonic_index >= LENGTH_OF_HARMONICS_ARRAY2)
	{
		// reset the coefficient index
		v->j = 0;

		// reset the harmonic index
		harmonic_index = 0;

		// lower the flag and stop calculation of coefficients
		v->CoeffCalcInProgressFlag = 0;

		// reset the state machine counter
		cleared_coeff_flag = 0;
	}
}


