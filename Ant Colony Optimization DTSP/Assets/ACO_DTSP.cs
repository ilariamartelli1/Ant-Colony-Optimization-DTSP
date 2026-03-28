using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.Linq;
using UnityEditor;

public class ACO_DTSP : Dubins { // Imports methods from Dubins class

	// Constant variables for Ranges in Inspector
	const int minNodes = 3;
	const int maxNodes = 100;
	const int minAnts = 2;
	const int maxAnts = 50;
	const float minRadius = 0.1f;
	const float maxRadius = 10.0f;
	const float minTimeDebug = 0.0f;
	const float maxTimeDebug = 5.0f;

	//----------------------------------------------------------------------//
	//----------------------------------------------------------------------//
	//                    Variables visible in Inspector                    //
	//----------------------------------------------------------------------//
	//----------------------------------------------------------------------//

	[Header("Sim Parameters")]
	[SerializeField] private int widthSimArea			= 100;
	[SerializeField] private int heightSimArea			= 80;

	[SerializeField] private bool rndNodes				= true;
	[SerializeField] private string rngSeed				= "111";
	[SerializeField] private bool rollRandomSeed		= false;

	[Range(minNodes,maxNodes)]
	[SerializeField] public int numNodes				= 25;
	[SerializeField] private bool setCircle				= false;
	[SerializeField] private bool setStar				= false;

	[Header("DTSP")]
	[SerializeField] public bool isTSP					= false;
	[SerializeField] public bool getFinalDubins			= true;
	[SerializeField] public bool isPulleyAlgorithm		= true;
	[Range(minRadius,maxRadius)]
	[SerializeField] public float radiusOfCurvature		= 2.0f;

	[Header("Visual Interface")]
	[SerializeField] private bool visualizeNodes		= true;
	[SerializeField] private bool visualizePheromone	= true;
	[SerializeField] private bool visualizeStraightPath	= false;

	[Header("TSP Library")]
	[SerializeField] private bool p01					= false;
	/*[SerializeField]*/ private bool ulysses22			= false;
	[SerializeField] private bool oliver30				= false;
	[SerializeField] private bool dantzig42				= false;
	/*[SerializeField]*/ private bool att48				= false;

	[Header("Ant Colony Optimization Settings")]

	// ACO Settings saved selection
	// Marco Dorigo:	m=10, beta=2, alpha=1, rho=0.1, q0=0.9, tau0=(n*L_nn)^-1
	// Camelia Pintea:	m=10, beta=5, alpha=1, rho=0.5, q0=0.5, tau0=(n*L_nn)^-1
	// Sebastian Lague:	m=20, beta=4, alpha=1, rho=0.1, q0=0.6, tau0=1
	// Set 1:			m=15, beta=2, alpha=1, rho=0.7, q0=0.7, tau0=(n*L_nn)^-1

	// Number of ants in each group
	[Range(minAnts,maxAnts)]
	[SerializeField] private int numAnts = 15;

	// Controls the extent to which ants will prefer nearby points
	// Too high and algorithm will essentially be a greedy search
	// Too low and the search will likely stagnate
	[SerializeField] private float dstPower = 2; // (beta)

	// Controls how likely each ant will be to follow the same paths as the ants before it
	// Too high and algorithm will keep searching the same path
	// Too low and it will search too many paths
	[SerializeField] private float pheromonePower = 1; // (alpha)

	// Proportion of pheromone that evaporates each step (pheromone decay parameter)
	[SerializeField] private float evaporationRate = 0.7f; // (rho)

	// Probability for which the best desiderability will be chosen (exploitation)
	[SerializeField] private float exploitationThreshold = 0.7f; // (q0)

	// Intensity of pheromone trail
	[SerializeField] private float pheromoneIntensity = 10; // (Q)

	// Initial pheromone strength along all paths
	// (Otherwise initial probabilities will all be zero)
	[SerializeField] private float initialPheromoneIntensity = 1;

	// Switches from Ant Optimization to Ant Colony Optimization
	// Affects pheromoneIntensity and initialPheromoneIntensity
	[SerializeField] private bool choosePheromoneIntensity;

	// Changes correction rule in the Local update
	// and re-initialize trails over an upper bound in the Global update
	[SerializeField] private bool reinforcingACS;

	[Header("Debug")]
	[SerializeField] private bool dstSetInfo = false;
	[Range(minTimeDebug,maxTimeDebug)]
	[SerializeField] private float timeBetwIterations = 0.0f;


	//----------------------------------------------------------------------//
	//----------------------------------------------------------------------//
	//                 Public variables hidden in Inspector                 //
	//----------------------------------------------------------------------//
	//----------------------------------------------------------------------//

	[HideInInspector] public float timer;
	[HideInInspector] public float bestTourDst;
	[HideInInspector] public float costDubinsPath;
	[HideInInspector] public int numNodesPlaced = 0;
	

	//----------------------------------------------------------------------//
	//----------------------------------------------------------------------//
	//                  Other private arrays and variables                  //
	//----------------------------------------------------------------------//
	//----------------------------------------------------------------------//

	// For SolveACODTSP
	private float[,] posNodes;
	private float[,] pheromoneTrails;
	private int[,] edgeInBestTour; // Adjacency matrix for a non-oriented graph
	private int[] iBestTour; // Indices of best tour found
	private float tau0;
	private float tauMax;

	// For CreateNodesFromInput
	private List<float>[] posNodesPlaced;

	// For the Dubins-Conversion Algorithm (Alternating or Pulley)
	private Vector3[,] pathDubinsArgs;
	private float[] headings;
	private int numPaths;

	// For the timer
	private float startTime = 0.0f;

	// Received from Button Handler
	private bool startAlgorithm = false;
	private bool stopAlgorithm = false;




	//----------------------------------------------------------------------//
	//----------------------------------------------------------------------//
	//                           Main and Methods                           //
	//----------------------------------------------------------------------//
	//----------------------------------------------------------------------//

	void Start() {
		StartCoroutine(CheckButtonStatus());
		StartCoroutine(CreateNodesFromInput());
		StartCoroutine(SolveACODTSPandWait());
	}

	IEnumerator CheckButtonStatus() {
		while (true) { // Loop to keep the method updated
			startAlgorithm = GameObject.Find("ButtonHandler").GetComponent<ButtonHandler>().startAlgorithm;
			stopAlgorithm = GameObject.Find("ButtonHandler").GetComponent<ButtonHandler>().stopAlgorithm;
			yield return null;
		}
	}

	IEnumerator CreateNodesFromInput() {
		posNodesPlaced = new List<float>[2]; // List array initializations
		posNodesPlaced[0] = new List<float>();
		posNodesPlaced[1] = new List<float>();
		while (!startAlgorithm) {
			// Runs only before the algorithm has started
			SceneView.duringSceneGui += OnSceneGUI;
			yield return null;
		}
	}

	void OnSceneGUI(SceneView sceneView) {
		if (startAlgorithm) return;
		float xMouse, yMouse;
		bool isSameNode = false;
		Event e = Event.current;
		// Click to place nodes
		if (e.type == EventType.MouseDown && e.button == 0) {
			Vector3 mousePosition = e.mousePosition;
			Ray ray = HandleUtility.GUIPointToWorldRay(mousePosition);
			(xMouse, yMouse) = (ray.origin.x, ray.origin.y);
			for (int i = numNodesPlaced-1; i >= 0; i--) {
				// The last placed node is more likely to be the same
				if (xMouse == posNodesPlaced[0][i] && yMouse == posNodesPlaced[1][i]) {
					isSameNode = true;
					break;
				}
			}
			if (isSameNode == false) {
				posNodesPlaced[0].Add(xMouse);
				posNodesPlaced[1].Add(yMouse);
				numNodesPlaced++;
			}
			e.Use(); // To avoid multiple triggers on the same click
		}
	}

	IEnumerator SolveACODTSPandWait() {
		while (true) { // Loop to keep the method updated
			if (!startAlgorithm) {
				yield return null; // pass
			}
			else {
				yield return SolveACODTSP();
			}
		}
	}

	(Vector3[,], float) AlternatingAlgorithm(int[] iTour) {
		Vector3[,] pathAADubinsArgs;
		float costAADubinsPath;
		int iLast = numNodes-1;
		numPaths = (int)Math.Ceiling((float)numNodes/2);
		headings = new float[numNodes];
		// Find Headings
		headings[0] = Nodes2HeadingAA(iTour[0], iTour[1]);
		for (int i = 1; i < iLast; i++) {
			if (i % 2 == 1)	// Odd
				headings[i] = headings[i-1];
			else			// Even
				headings[i] = Nodes2HeadingAA(iTour[i], iTour[i+1]);
		}
		headings[iLast] = (numNodes % 2 == 0) ? headings[iLast-1] : Nodes2HeadingAA(iTour[iLast], iTour[0]);
		
		(pathAADubinsArgs, costAADubinsPath) = PathPlanner(iTour);
		return (pathAADubinsArgs, costAADubinsPath);
	}

	(Vector3[,], float) PulleyAlgorithm(int[] iTour) {
		Vector3[,] pathPADubinsArgs;
		float costPADubinsPath;
		int iLast = numNodes-1;
		numPaths = numNodes;
		headings = new float[numNodes];
		// Find Headings
		headings[0] = Nodes2HeadingPA(iTour[iLast], iTour[0], iTour[1]);
		for (int i = 1; i < iLast; i++) {
			headings[i] = Nodes2HeadingPA(iTour[i-1], iTour[i], iTour[i+1]);
		}
		headings[iLast] = Nodes2HeadingPA(iTour[iLast-1], iTour[iLast], iTour[0]);
		
		(pathPADubinsArgs, costPADubinsPath) = PathPlanner(iTour);
		return (pathPADubinsArgs, costPADubinsPath);
	}

	(Vector3[,], float) PathPlanner(int[] iTour) {
		Vector3[,] pathDubinsArgs = new Vector3[numPaths,10];
		Vector3[] pathDubinsVisualize;
		float[] startCoord, endCoord;
		float xPosI, yPosI, thetaI; // Initial conditions
		float xPosD, yPosD, thetaD; // Desired target
		float costDubinsPath = 0, cost;
		int nCurves = 0;
		for (int i = 0; i < numNodes; i++) {
			int j = (i == numNodes-1) ? 0 : i+1; // Changes if last iteration
			if (headings[i] != headings[j]) { // Dubins unicycle curve
				(xPosI, yPosI) = (posNodes[iTour[i],0], posNodes[iTour[i],1]);
				thetaI = headings[i];
				(xPosD, yPosD) = (posNodes[iTour[j],0], posNodes[iTour[j],1]);
				thetaD = headings[j];
				startCoord = new float[] {xPosI, yPosI, thetaI};
				endCoord = new float[] {xPosD, yPosD, thetaD};
				(pathDubinsVisualize, cost) = GetDubinsPath(startCoord, endCoord, radiusOfCurvature);
				pathDubinsArgs = SetRow(pathDubinsArgs, pathDubinsVisualize, nCurves);
				nCurves++;
			} else {
				cost = GetDistance(iTour[i], iTour[j]); // Straight line
			}
			costDubinsPath += cost;
		}
		return (pathDubinsArgs, costDubinsPath);
	}

	IEnumerator SolveACODTSP() {
		float dstCheck = Mathf.Infinity; // Distance found should always decrease inside algorithm while loop
		float Q = (choosePheromoneIntensity) ? pheromoneIntensity : evaporationRate;
		InitializeACO();
		float deltaTau = tau0; // for the Local update
		float bestTourDstLast = 1 / ((1-evaporationRate) * tauMax); // (Lnn)
		startTime = 0.0f;

		// Loop on best paths found (ITERATION)
		while (!stopAlgorithm) {
			int[,,] edgeInTour = new int[numNodes,numNodes,numAnts]; // Default all zeros
			int[,] iTour = new int[numNodes,numAnts];
			bool[,] workingMemory = new bool[numNodes,numAnts]; // Default all false
			float[] dstTot = new float[numAnts]; // Default all zeros
			int[] iStartNode = new int[numAnts];
			int[] iCurrentNode = new int[numAnts];
			int[] iNextNode = new int[numAnts];

			if (startTime == 0.0f) startTime = Time.time; // Time of activation

			// Uses two different methods to randomize initial nodes for the ants
			int[] nodeIndices = new int[numAnts];
			if (numNodes >= numAnts) {	// without repetition
				nodeIndices = Enumerable.Range(0,numNodes).ToArray();
				nodeIndices = Shuffle(nodeIndices);
			} else {					// with repetition
				System.Random rnd = new System.Random();
				for (int i = 0; i < numAnts; i++)
					nodeIndices[i] = rnd.Next(0,numNodes);
			}

			// Starting nodes for the ants
			for (int i = 0; i < numAnts; i++) {
				iCurrentNode[i] = nodeIndices[i];
				iStartNode[i] = iCurrentNode[i]; // Saves starting nodes to close the paths
			}
			
			// Loop on every edge (STEP)
			for (int step = 0; step < numNodes; step++) { // (num of edges in a path = numNodes)
				// Executes the current STEP for every nth ant
				for (int n = 0; n < numAnts; n++) {
					workingMemory[iCurrentNode[n],n] = true; // Excludes nodes already visited

					if (step == numNodes-1) // Last STEP (closes the path)
						iNextNode[n] = iStartNode[n];
					else
						iNextNode[n] = ChooseNextNode(iCurrentNode[n], GetColumn(workingMemory,n), numNodes-1-step);
					
					if (isTSP) {
						dstTot[n] += GetDistance(iCurrentNode[n], iNextNode[n]); // Length of paths taken by each ant
					}
					edgeInTour[iCurrentNode[n],iNextNode[n],n] = 1;
					edgeInTour[iNextNode[n],iCurrentNode[n],n] = 1; // For symmetry
					
					iTour[step,n] = iCurrentNode[n]; // Saves tour indices
				}	
				for (int n = 0; n < numAnts; n++) { // Other loop to avoid bias between ants on the same STEP
					// ACS Local Updating Rule
					var (i, j) = (iCurrentNode[n], iNextNode[n]);
					if (reinforcingACS) deltaTau = 1/(numNodes*bestTourDstLast); // 1/(n*L_gb_last)
					pheromoneTrails[i,j] = (1 - evaporationRate) * pheromoneTrails[i,j] + evaporationRate * deltaTau;
					pheromoneTrails[j,i] = (1 - evaporationRate) * pheromoneTrails[j,i] + evaporationRate * deltaTau; // For symmetry
					
					iCurrentNode[n] = iNextNode[n]; // Updates current node for every ant
				}
			}
			// Checks which ant took the best path
			int iWin = -1;
			for (int i = 0; i < numAnts; i++) {
				if (!isTSP) {
					Vector3[,] pathDubinsArgsTemp;
					if (isPulleyAlgorithm)
						(pathDubinsArgsTemp, dstTot[i]) = PulleyAlgorithm(GetColumn(iTour,i));
					else
						(pathDubinsArgsTemp, dstTot[i]) = AlternatingAlgorithm(GetColumn(iTour,i));
				}
				if (dstTot[i] < dstCheck) {
					dstCheck = dstTot[i];
					iWin = i;
				}
			}
			// Updates best tour indices and edges
			if (iWin != -1) { // Found a better global path
				for (int i = 0; i < numNodes; i++) {
					iBestTour[i] = iTour[i,iWin];
					for (int j = 0; j < numNodes; j++)
						edgeInBestTour[i,j] = edgeInTour[i,j,iWin];
				}
			}
			bestTourDst = dstCheck; // (L_gb)
			bestTourDstLast = bestTourDst; // (L_gb_last)
			tauMax = 1 / ((1-evaporationRate) * bestTourDst); // update tauMax
			if (!isTSP) {
				if (isPulleyAlgorithm)
					(pathDubinsArgs, costDubinsPath) = PulleyAlgorithm(iBestTour);
				else
					(pathDubinsArgs, costDubinsPath) = AlternatingAlgorithm(iBestTour);
			}

			// ACS Global Updating Rule
			for (int i = 0; i < numNodes; i++) {
				for (int j = 0; j < numNodes; j++) {
					pheromoneTrails[i,j] = (1 - evaporationRate) * pheromoneTrails[i,j];
					if (edgeInBestTour[i,j] == 1) // 1 if edge is present
						pheromoneTrails[i,j] += Q / bestTourDst;
					if (reinforcingACS && pheromoneTrails[i,j] > tauMax) // upper bound
						pheromoneTrails[i,j] = tau0; // re-initialize
				}
			}

			timer = Time.time - startTime; // Updates timer after each ITERATION
			yield return new WaitForSeconds(timeBetwIterations); // waits if in debug mode
		}

		// Only reachable when the ACO algorithm has been stopped
		if (isTSP && getFinalDubins) {
			if (isPulleyAlgorithm)
				(pathDubinsArgs, costDubinsPath) = PulleyAlgorithm(iBestTour);
			else
				(pathDubinsArgs, costDubinsPath) = AlternatingAlgorithm(iBestTour);
		}
	}

	int ChooseNextNode(int iCurrentNode, bool[] alreadyVisited, int numNodesToVisit) {
		float[] desNodesToVisit = new float[numNodesToVisit];
		int[] iNodesToVisit = new int[numNodesToVisit];
		float dst, pheromoneStrength, desirability, desBest = -1, desTot = 0;
		int iWin = -1, potentialNodes = 0;

		// Pseudo-Random-Proportional Rule
		System.Random rnd = new System.Random();
		float q = (float)rnd.NextDouble();
		float q0 = exploitationThreshold;
		bool chooseBest = (q <= q0) ? true : false; // exploitation VS biased exploration
		
		for (int iPotentialNext = 0; iPotentialNext < numNodes; iPotentialNext++) {
			if (alreadyVisited[iPotentialNext] == false) {
				
				dst = GetDistance(iCurrentNode, iPotentialNext);
				pheromoneStrength = pheromoneTrails[iCurrentNode, iPotentialNext];
				// desirability = attractiveness (eta) * trail level (tau)
				desirability = Mathf.Pow(1 / dst, dstPower) * Mathf.Pow(pheromoneStrength, pheromonePower);
				if (desirability == 0) // Avoids limit case
					desirability = 1.4e-45f; // float positive denormalized minimum (1.4*10^-45)
				
				// Exploitation
				if (chooseBest && desirability > desBest) {
					iWin = iPotentialNext;
					desBest = desirability;
				}
				if (!chooseBest) {
					iNodesToVisit[potentialNodes] = iPotentialNext; // Stores indices
					desNodesToVisit[potentialNodes] = desirability; // Stores desirability
					desTot += desirability; // Stores normalizer for pdf
					potentialNodes++;
				}
			}
		}
		// Biased Exploration
		if (!chooseBest) {
			float[] pdf = new float[numNodesToVisit];
			float rndCheck, CDF = 0;
			// Generates discrete probability distribution function (pdf)
			for (int i = 0; i < numNodesToVisit; i++) {
				pdf[i] = desNodesToVisit[i] / desTot; // Normalized
			}
			rndCheck = (float)rnd.NextDouble();
			// Uses discrete cumulative distribution function (CDF)
			// to work out weighted probabilities
			for (int i = 0; i < numNodesToVisit; i++) {
				CDF += pdf[i];
				if (rndCheck <= CDF)
					return iNodesToVisit[i];
			}
			// Unsolved case due to float approssimation
			return iNodesToVisit[numNodesToVisit-1];
		}
		return iWin;
	}

	void InitializeACO() {
		if (p01) {
			(posNodes, numNodes) = SetConfigurationP01(posNodes, numNodes);
		} else if (ulysses22) {
			(posNodes, numNodes) = SetConfigurationUlysses22(posNodes, numNodes);
		} else if (oliver30) {
			(posNodes, numNodes) = SetConfigurationOliver30(posNodes, numNodes);
		} else if (dantzig42) {
			(posNodes, numNodes) = SetConfigurationDantzig42(posNodes, numNodes);
		} else if (att48) {
			(posNodes, numNodes) = SetConfigurationAtt48(posNodes, numNodes);
		} else if (setCircle) {
			(posNodes, numNodes) = SetConfigurationCircle(posNodes, numNodes);
		} else if (setStar) {
			(posNodes, numNodes) = SetConfigurationStar(posNodes, numNodes);

		} else if (rndNodes || numNodesPlaced < 3) {
			// Choose random nodes
			if (rollRandomSeed) rngSeed = Time.realtimeSinceStartup.ToString();
			System.Random rnd = new System.Random(rngSeed.GetHashCode());
			posNodes = new float[numNodes,2];
			for (int i = 0; i < numNodes; i++) {
				posNodes[i,0] = rnd.Next(-widthSimArea/2, widthSimArea/2) + (float)Math.Round(rnd.NextDouble(), 2);
				posNodes[i,1] = rnd.Next(-heightSimArea/2, heightSimArea/2) + (float)Math.Round(rnd.NextDouble(), 2);
			}

		} else {
			posNodes = new float[numNodesPlaced,2];
			for (int i = 0; i < numNodesPlaced; i++) {
				posNodes[i,0] = posNodesPlaced[0][i];
				posNodes[i,1] = posNodesPlaced[1][i];
			}
			numNodes = numNodesPlaced;
		}

		// Initialize variables
		float Lnn = GetLengthNearestNeighbor();
		tau0 = (choosePheromoneIntensity) ? initialPheromoneIntensity : 1 / (numNodes * Lnn);
		tauMax = 1 / ((1-evaporationRate) * Lnn); // for RACS (Reinforcing ACS)
		pheromoneTrails = new float[numNodes,numNodes];
		edgeInBestTour = new int[numNodes,numNodes]; // Default all zeros
		iBestTour = new int[numNodes];
		for (int i = 0; i < numNodes; i++) {
			for (int j = 0; j < numNodes; j++) {
				pheromoneTrails[i,j] = tau0;
			}
		}

		if (dstSetInfo) {
			// Show additional info in the Console log about the current node set
			float dst, dstAvg = 0, dstCheck = Mathf.Infinity;
			for (int i = 0; i < numNodes; i++) {
				for (int j = 0; j < numNodes; j++) {
					if (i != j) {
						dst = GetDistance(i,j);
						dstAvg += dst;
						if (dst < dstCheck)
							dstCheck = dst;
					}
				}
			}
			// Total number of edges in a complete graph of N vertices = (n*(n - 1))/2
			dstAvg /= numNodes*(numNodes-1); // *2 because dstAvg has both i*j and j*i edges
			Debug.Log($"Smallest distance between nodes = " + dstCheck.ToString("0.000") + "\t\t" +
						"Average distance between nodes = " + dstAvg.ToString("0.000"));
		}
	}

	float GetLengthNearestNeighbor() {
		bool[] alreadyVisited = new bool[numNodes]; // Default all false
		float dstCheck = Mathf.Infinity, dstTot = 0;
		int iStartNode = 0, iCurrentNode = 0, iWin = -1, iNextNode;
		for (int step = 0; step < numNodes; step++) {
			alreadyVisited[iCurrentNode] = true;
			if (step == numNodes-1) { // Last STEP (closes the path)
				iNextNode = iStartNode;
			} else {
				for (int iPotentialNext = 0; iPotentialNext < numNodes; iPotentialNext++) {
					if (alreadyVisited[iPotentialNext] == false) {
						float dst = GetDistance(iCurrentNode, iPotentialNext);
						if (dst < dstCheck) {
							dstCheck = dst;
							iWin = iPotentialNext;
						}
					}
				}
				iNextNode = iWin;
			}
			dstTot += GetDistance(iCurrentNode, iNextNode);
			iCurrentNode = iNextNode;
		}
		return dstTot;
	}

	float Nodes2HeadingAA(int iNode1, int iNode2) {
		float x1,x2,y1,y2,heading;
		(x1, y1) = (posNodes[iNode1,0], posNodes[iNode1,1]);
		(x2, y2) = (posNodes[iNode2,0], posNodes[iNode2,1]);
		heading = Mathf.Atan2(y2-y1, x2-x1);
		return heading;
	}

	float Nodes2HeadingPA(int iNode1, int iNode2, int iNode3) {
		float x1,y1,y2,x2,x3,y3,xf1,yf1,k1,k2;
		float a1,b1,c1,a2,b2,c2,a3,b3,c3,heading;
		(x1, y1) = (posNodes[iNode1,0], posNodes[iNode1,1]);
		(x2, y2) = (posNodes[iNode2,0], posNodes[iNode2,1]);
		(x3, y3) = (posNodes[iNode3,0], posNodes[iNode3,1]);
		(a1,b1,c1) = Points2Line(x1,y1,x2,y2); // backward
		(a2,b2,c2) = Points2Line(x2,y2,x3,y3); // forward
		k1 = Mathf.Sqrt(Mathf.Pow(a1,2) + Mathf.Pow(b1,2));
		k2 = Mathf.Sqrt(Mathf.Pow(a2,2) + Mathf.Pow(b2,2));
		// This assures that the possible error is on a4,b4,c4
		if (a1 == -a2 && b1 == -b2 && c1 == -c2) { a2 = -a2; b2 = -b2; c2 = -c2; }
		// Two possible bisectors of angles generated by intersecting lines
		(a3,b3,c3) = ((a1/k1+a2/k2),(b1/k1+b2/k2),(c1/k1+c2/k2));
		(xf1,yf1) = PointOnPerpendicular(a3, b3, c3, x3, y3); // forward bisector 1
		// If the 3 nodes are on the same line there is no second bisector
		heading = Mathf.Atan2(yf1-y2, xf1-x2);
		if (!(a1 == a2 && b1 == b2 && c1 == c2)) {
			float a4,b4,c4,xb1,yb1,xf2,yf2,dist;
			// Second bisector
			(a4,b4,c4) = ((a1/k1-a2/k2),(b1/k1-b2/k2),(c1/k1-c2/k2));
			(xb1,yb1) = PointOnPerpendicular(a3, b3, c3, x1, y1); // backward bisector 1
			(xf2,yf2) = PointOnPerpendicular(a4, b4, c4, x3, y3); // forward bisector 2
			dist = Points2Dist(xb1,yb1,xf1,yf1);
			if (dist < Points2Dist(x2,y2,xb1,yb1) || dist < Points2Dist(x2,y2,xf1,yf1))
				heading = Mathf.Atan2(yf2-y2, xf2-x2);
		}
		return heading;
	}

	float GetDistance(int a, int b) {
		float x1,x2,y1,y2;
		(x1, y1) = (posNodes[a,0], posNodes[a,1]);
		(x2, y2) = (posNodes[b,0], posNodes[b,1]);
		return Mathf.Sqrt(Mathf.Pow(x1 - x2, 2) + Mathf.Pow(y1 - y2, 2));
	}

	void OnDrawGizmos() {
		Vector3 v = Vector3.forward; // All the gizmos face forward in a 2D plane
		if (posNodes != null) {
			// Draw nodes
			for (int i = 0; i < numNodes; i++) {
				if (visualizeNodes) {
					Vector3 pos = new Vector3(posNodes[i,0], posNodes[i,1], 0);
					Handles.color = Color.white;
					Handles.DrawSolidDisc(pos, v, 1);
				}
				if (startAlgorithm) {
					// Draw edges
					for (int j = 0; j < numNodes; j++) {
						Vector3 pos_i = new Vector3(posNodes[i,0], posNodes[i,1], 0);
						Vector3 pos_j = new Vector3(posNodes[j,0], posNodes[j,1], 0);
						// Draw pheromone trails
						if (!stopAlgorithm && visualizePheromone) {
							float alpha = pheromoneTrails[i,j];
							alpha *= (choosePheromoneIntensity) ? 0.03f*pheromoneIntensity : 300;
							if (reinforcingACS) alpha *= 100;
							Handles.color = new Color(1, 1, 1, alpha); // Fading white
							Handles.DrawLine(pos_i, pos_j);
						}
						// Draw best tour (Straight lines)
						if ((isTSP && !getFinalDubins) || (isTSP && getFinalDubins && !stopAlgorithm) || visualizeStraightPath) {
							if (edgeInBestTour[i,j] == 1) { // 1 if edge is present
								Handles.color = ((isTSP && !getFinalDubins) || (isTSP && getFinalDubins && !stopAlgorithm)) ? Color.white : Color.red;
								Handles.DrawLine(pos_i, pos_j);
							}
						}
					}
				}
			}
			if ((!isTSP && headings != null) || (isTSP && getFinalDubins && stopAlgorithm)) {
				// Draw Dubins Path
				Handles.color = Color.white;
				int nCurve = 0;
				for (int i = 0; i < numNodes; i++) {
					int j = (i == numNodes-1) ? 0 : i+1; // Closes path
					if (headings[i] != headings[j]) {	// Dubins unicycle curve
						Vector3 center1, center2, center3, from1, from2, from3, endArc1, endArc3, arcs, strSeg;
						float arc1, arc2, arc3, r = radiusOfCurvature;
						Vector3[] pathDubinsArgsRow = GetRow(pathDubinsArgs, nCurve);
						(center1, center2, center3, from1, from2, from3, endArc1, endArc3, arcs, strSeg) = UnpackArray10(pathDubinsArgsRow);
						(arc1, arc2, arc3) = (arcs[0], arcs[1], arcs[2]);
						bool straightSegment = (strSeg[0] == 1.0f) ? true : false;
						// First segment (always a curve if present)
						Handles.DrawWireArc(center1, v, from1, arc1, r);
						// Second segment (straight line or curve)
						if (straightSegment)
							Handles.DrawLine(endArc1, endArc3);
						else
							Handles.DrawWireArc(center2, v, from2, arc2, r);
						// Third segment (always a curve if present)
						Handles.DrawWireArc(center3, v, from3, arc3, r);
						nCurve++;
					} else {							// Straight line
						Vector3 pos_i = new Vector3(posNodes[iBestTour[i],0], posNodes[iBestTour[i],1], 0);
						Vector3 pos_j = new Vector3(posNodes[iBestTour[j],0], posNodes[iBestTour[j],1], 0);
						Handles.DrawLine(pos_i, pos_j);
					}
				}
			}
		}
		else { // Draw nodes placed before the start of the algorithm
			for (int i = 0; i < numNodesPlaced; i++) {
				Vector3 pos = new Vector3(posNodesPlaced[0][i], posNodesPlaced[1][i], 0);
				Handles.color = Color.white;
				Handles.DrawSolidDisc(pos, v, 1);
			}
		}
	}
}
