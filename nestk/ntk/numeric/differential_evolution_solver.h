// Differential Evolution Solver Class
// Based on algorithms developed by Dr. Rainer Storn & Kenneth Price
// Written By: Lester E. Godwin
//             PushCorp, Inc.
//             Dallas, Texas
//             972-840-0208 x102
//             godwin@pushcorp.com
// Created: 6/8/98
// Last Modified: 6/8/98
// Revision: 1.0

#ifndef NTK_NUMERIC_DIFFERENTIAL_EVOLUTION_SOLVER_H
#define NTK_NUMERIC_DIFFERENTIAL_EVOLUTION_SOLVER_H

#include <ntk/core.h>

namespace ntk
{

class DifferentialEvolutionSolver;

typedef void (DifferentialEvolutionSolver::*StrategyFunction)(int);

class DifferentialEvolutionSolver
{
public:
  enum StategyType
  {
    stBest1Exp = 0,
    stRand1Exp = 1,
    stRandToBest1Exp = 2,
    stBest2Exp = 3,
    stRand2Exp = 4,
    stBest1Bin = 5,
    stRand1Bin = 6,
    stRandToBest1Bin = 7,
    stBest2Bin = 8,
    stRand2Bin = 9,
  };

public:
  DifferentialEvolutionSolver(int dim,int popSize);
  ~DifferentialEvolutionSolver(void);

	// Setup() must be called before solve to set min, max, strategy etc.
  void Setup(double min[],double max[], StategyType deStrategy,
             double diffScale,double crossoverProb);

  void Reset();

	// Solve() returns true if EnergyFunction() returns true.
	// Otherwise it runs maxGenerations generations and returns false.
	virtual bool Solve(int maxGenerations);

	// EnergyFunction must be overridden for problem to solve
	// testSolution[] is nDim array for a candidate solution
	// setting bAtSolution = true indicates solution is found
	// and Solve() immediately returns true.
  virtual double EnergyFunction(double testSolution[],bool &bAtSolution) = 0;

	int Dimension(void) { return(nDim); }
	int Population(void) { return(nPop); }

	// Call these functions after Solve() to get results.
	double Energy(void) { return(bestEnergy); }
	double *Solution(void) { return(bestSolution); }

	int Generations(void) { return(generations); }

protected:
	void SelectSamples(int candidate,int *r1,int *r2=0,int *r3=0,
												int *r4=0,int *r5=0);
	double RandomUniform(double min,double max);

	int nDim;
	int nPop;
	int generations;
	int strategy;
	StrategyFunction calcTrialSolution;
	double scale;
	double probability;

	double trialEnergy;
	double bestEnergy;

	double *trialSolution;
	double *bestSolution;
	double *popEnergy;
	double *population;

private:
	void Best1Exp(int candidate);
	void Rand1Exp(int candidate);
	void RandToBest1Exp(int candidate);
	void Best2Exp(int candidate);
	void Rand2Exp(int candidate);
	void Best1Bin(int candidate);
	void Rand1Bin(int candidate);
	void RandToBest1Bin(int candidate);
	void Best2Bin(int candidate);
	void Rand2Bin(int candidate);
};

} // ntk

#endif // NTK_NUMERIC_DIFFERENTIAL_EVOLUTION_SOLVER_H
