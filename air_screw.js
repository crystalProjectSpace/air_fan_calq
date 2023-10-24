'use strict'
/**
* This is a program to calculate thrust and power performance of airscrew assembly
* It divides fan assembly into separate blades
* Each fan blade is divided into separate areas
* A distribution of local air velocities and AoA is calculated for each area
* This data is used to calculate local lift (thrust) and drag across single blade
* Airfoil aerodynamics Cy = Cy(alpha), Cx = Cx(alpha) are used to a/d forces calculation
* drag distribution is used to calculate drag torque acting on single fan blade
* Required power on fan is equal to total torque acting on each fan blade multiplied to cyclic frequency
* Actual power should be equal to required torque multiplied to motor and shaft efficiency
*-------------------------------------------------------------------------------------------
* Calculation doesnt include influence of cowl on fan efficiency (reduced inductive drag on blade due to reduced flow spillage from lower surface to upper) 
*/

/**
* @description Одномерная линейная интерполяция
*/
const interp = function(VX, VY, X) {
	const size = VX.length
	let i1 = 1
	for(let i = 0; i < size; i++) {
		if (VX[i1] > X) {
			const dX = X - VX[i]
			const k = (VY[i1] - VY[i]) / (VX[i1] - VX[i])
			return VY[i] + k * dX
		}
		i1++
	}
}
/**
* @description получить распределение локальных скоростей и углов атаки по лопасти винта
*/
const getSpeedDistribution = function(RPM, R, v0, N) {
	const omega = 2 * Math.PI * RPM
	const res = []

	const dR = R / N
	let localR = dR * 0.5
	
	for(let i = 0; i < N; i++) {
		const localV = omega * localR
		const totalV = Math.sqrt(v0 * v0 + localV * localV)
		const localAoA = -Math.atan(v0 / localV)
		
		res.push({
			range: localR,
			localVelocity: totalV,
			localAoA: localAoA * 57.3,
		})
		
		localR += dR
	}
	
	return res
}
/**
* @description получить силы, действующие на отдельную лопасть
* @param RX - длины элементарных участков
* @param AX - угловая крутка профиля на каждом участке
* @param SX - площадь каждого элементарного участка
* @param Cx0 - коэф.сопротивления профиля при нулевой подъемной силе
* @param RPM - частота вращения (Гц)
* @param v0 -  скорость невозмущенного потока
* @param Ro - плотность невозмущенного потока
* @param AX_CY - опорные углы атаки для интерполяции АДХ
* @param CY - коэф. углов подъемной силы
* @param POLAR - коэф.отвала поляры
* @param VX - опорные значения скорости
* @param dAVX - опорный угол для изменения шага винта в зависимости от скорости
*/
const getSingleBladeForces = function(RX, AX, SX, Cx0, RPM, v0, Ro, AX_CY, CY, POLAR, VX = [], dAVX = []) {
	const N = RX.length
	const R = RX[N - 1]
	
	let dAlpha = VX.length ? interp(VX, dAVX, v0) : 0
	
	const speedDistribution = getSpeedDistribution(RPM, R, v0, N)
	
	const forceDistribution = []
	
	let totalThrust = 0
	let totalDrag = 0
	let totalTorque = 0
	
	for(let i = 0; i < N; i++) {
		const {	range, localVelocity, localAoA } = speedDistribution[i]
		const localQ = 0.5 * Ro * localVelocity * localVelocity
		const bladeSectionArea = interp(RX, SX, range)
		const bladeSectionAoA = interp(RX, AX, range)
		const localQS = bladeSectionArea * localQ
		const totalAoA = bladeSectionAoA + localAoA + dAlpha
		const Cya = interp(AX_CY, CY, totalAoA)
		const aPolar = interp(AX_CY, POLAR, totalAoA)
		const Cxa = Cx0 + aPolar * Cya * Cya
		const localThrust = Cya * localQS
		const localDrag = Cxa * localQS
		const localTorque = localDrag * range
		
		forceDistribution.push({
			thrust: localThrust,
			drag: localDrag,
			torque: localTorque,
		})
		
		totalThrust += localThrust
		totalDrag += totalDrag
		totalTorque += localTorque
	}
	
	return {
		speedDistribution,
		forceDistribution,
		summary: {
			totalThrust,
			totalDrag,
			totalTorque,
		}
	}	
}
/**
* @description получить интегральные характеристики винта
* @returns значения тяги/мощности/момента для каждой точки по скорости полета
*/
const getSpeedThrustPerformance = function(initData) {
	const { nBlade, RX, SX, AX, VX = [], dAVX = [] } = initData.geometry
	const { AX_CY, Cx0, CY, POLAR } = initData.aerodynamics
	const { RPM, v0, v1, nV, Ro } = initData.environment
	
	const omega = 2 * Math.PI * RPM
	const rad = RX[RX.length - 1]
	const area = Math.PI * rad * rad
	const dV = (v1 - v0) / nV
	const res = []
	const _nV = nV + 1

	let V = v0
	
	for(let i = 0; i < _nV; i++) {
		const bladePerformance = getSingleBladeForces(RX, AX, SX, Cx0, RPM, V, Ro, AX_CY, CY, POLAR, VX, dAVX)
		const { totalThrust, totalDrag, totalTorque } = bladePerformance.summary
	
		const Q = 0.5 * Ro * V * V
	
		const thrust = totalThrust * nBlade
		const torque = totalTorque * nBlade
		const power = torque * omega
		const thrustCoeff = thrust / (area * Q)
	
		res.push({
			V: V * 3.6,
			thrust,
			torque,
			power,
			thrustCoeff,
			thrust2power: thrust / power
		})
		
		V += dV
	}
	
	return res
}
// Начало тестового примера (16-дюймовый винт)
const baseRPM = 95
// срезы по длине винта
const testRange =  [0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18, 0.2]
// угловая крутка профиля по длине лопасти
const testAlphaDistrib = [12.5, 12.5, 11.5, 9.5, 8.5, 8.5, 5.5, 5, 5, 5, 5]
// площади по срезам винта
const testAreaDistrib = [0.0002, 0.0002, 0.0002, 0.0002, 0.0002, 0.0002, 0.0002, 0.0002, 0.0002, 0.0002, 0.0002]
// тестовые скорости
const testV          = [0, 25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 275]
// отклонение лопасти винта от скорости
const testDeltaBlade = [6, 10, 14, 18, 22,  26,  30,  34,  38,  42,  46,  50]
// расчетный диапазон местных углов атаки
const testFoilAlpha = [-60, -30,  -20,   -15,   -10,  -8,    -6,    -4,      0,     4,    6,    8,     10,   15,    20,    30, 60]
// коэф.подъемной силы профиля от угла атаки
const testFoilCy =    [-0.4, -0.8, -1.05, -0.95, -0.8, -0.64, -0.48, -0.32,   0,     0.32, 0.48, 0.64,  0.8,  0.95,  1.05,  0.8, 0.4]
// коэф.поляры для профиля
const testFoilPolar = [0.75, 0.25, 0.175, 0.135, 0.105, 0.085, 0.07, 0.065, 0.065, 0.065,  0.07, 0.085,0.105,0.135, 0.175, 0.25, 0.75]
// коэф.профильного сопротивления лопасти
const Cx_0 = 0.045
// количество лопастей
const testNBlade = 6

const testAirScrew = {
	geometry: {
		nBlade: testNBlade,
		RX: testRange,
		SX: testAreaDistrib,
		AX: testAlphaDistrib,
		VX: testV,
		dAVX: testDeltaBlade
	},
	aerodynamics: {
		AX_CY: testFoilAlpha,
		Cx0: Cx_0,
		CY: testFoilCy,
		POLAR: testFoilPolar,
	},
	environment: {
		RPM: baseRPM,
		v0: 0,
		v1: 45,
		nV: 15,
		Ro: 1.225
	}
}

const testPerf = getSpeedThrustPerformance(testAirScrew)

console.log(testPerf)
/**
* { @link - used data from this page to test program output https://rcopen.com/forum/f20/topic171881/56 }
*/