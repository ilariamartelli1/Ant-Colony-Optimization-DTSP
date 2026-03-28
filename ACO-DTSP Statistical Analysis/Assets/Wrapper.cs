using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.Linq;
using UnityEditor;

public class Wrapper : ACODTSP_Wrapper { // Imports methods from ACODTSP_Wrapper class

	// For Statistical Analysis
	private float[] timerList;
	private float[] bestTourList;
	private float[] costDubinsList;
	[HideInInspector] public int countStat = 0;
	private bool solveStopped = false;
	private bool drawOnStop = false;
	private bool endReps = false;
	private bool hasStatStarted;
	
	// Received from Button Handler
	private bool startStat = false;


	void Start() {
		StartCoroutine(CheckButtonStatus());
		StartCoroutine(SolveACODTSPWrapper());
	}

	IEnumerator CheckButtonStatus() {
		while (!startStat) {
			startStat = GameObject.Find("ButtonHandler").GetComponent<ButtonHandler>().startAlgorithm;
			yield return null;
		}
	}

	void DisplayStat() {
		float timerAverage = 0, bestTourAverage = 0, costDubinsAverage = 0;
		for (int i = 0; i < numStatRep; i++) {
			timerAverage += timerList[i];
			bestTourAverage += bestTourList[i];
			costDubinsAverage += costDubinsList[i];
		}
		timerAverage /= numStatRep;
		bestTourAverage /= numStatRep;
		costDubinsAverage /= numStatRep;
		Debug.Log($"numIterations = " + numIterations.ToString() + "\t\t" +
					"bestTourAverage = " + bestTourAverage.ToString("0.000") + "\t\t" +
					"costDubinsAverage = " + costDubinsAverage.ToString("0.000") + "\t\t" +
					"timerAverage = " + timerAverage.ToString("0.00"));
	}

	void WaitAndUpdate() {
		if (hasStatStarted && countStat < numStatRep)
			countStat++;
		if (countStat < numStatRep) // refresh for next Repetition
			solveStopped = false;
		if (countStat >= numStatRep && !endReps) {
			// Show statistics when the Repetitions have terminated
			DisplayStat();
			endReps = true;
		}
	}

	IEnumerator SolveACODTSPWrapper() {
		
		timerList = new float[numStatRep];
		bestTourList = new float[numStatRep];
		costDubinsList = new float[numStatRep];
		numIterations = numIterFrom;

		while (true) { // Loop to keep the method updated
			
			if (endReps == true) {
				if (numIterations + numIterStep > numIterTo) { break; }
				numIterations += numIterStep;
				countStat = 0;
				endReps = false;
			}

			if (!startStat || solveStopped) {
				hasStatStarted = solveStopped == true; // it stopped at least once
				WaitAndUpdate();
				if (!pureStat || !startStat) {
					if (hasStatStarted && showResult) {
						drawOnStop = true;
						yield return new WaitForSeconds(1.0f); // Wait to show results of current Rep
					} else
						yield return null; // pass
				}
			}
			else {
				drawOnStop = false;
				yield return SolveACODTSP();
				
				// Save statistics of current Rep
				timerList[countStat] = timer;
				bestTourList[countStat] = bestTourDst;
				costDubinsList[countStat] = costDubinsPath;
				solveStopped = true; // goes to waiting loop
			}
		}
	}

	void OnDrawGizmos() {
		
		if (pureStat) { return; }

		Vector3 v = Vector3.forward; // All the gizmos face forward in a 2D plane
		if (posNodes != null) {
			// Draw nodes
			for (int i = 0; i < numNodes; i++) {
				if (visualizeNodes) {
					Vector3 pos = new Vector3(posNodes[i,0], posNodes[i,1], 0);
					Handles.color = Color.white;
					Handles.DrawSolidDisc(pos, v, 1);
				}
				if (drawOnStop) {
					// Draw edges
					for (int j = 0; j < numNodes; j++) {
						Vector3 pos_i = new Vector3(posNodes[i,0], posNodes[i,1], 0);
						Vector3 pos_j = new Vector3(posNodes[j,0], posNodes[j,1], 0);
						// Draw best tour (Straight lines)
						if ((isTSP && !getFinalDubins) || visualizeStraightPath) {
							if (edgeInBestTour[i,j] == 1) { // 1 if edge is present
								Handles.color = (isTSP && !getFinalDubins) ? Color.white : Color.red;
								Handles.DrawLine(pos_i, pos_j);
							}
						}
					}
				}
			}
			if (drawOnStop && ((!isTSP && headings != null) || (isTSP && getFinalDubins))) {
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
