using UnityEngine;
using UnityEngine.UI;
using System;
using TMPro;

public class TextManager : MonoBehaviour {

	public TextMeshProUGUI text;

	private bool startAlgorithm;
	private bool stopAlgorithm;
	private bool isTSP;
	private bool getFinalDubins;

	private float duration = 0;
	private float costTSP;
	private float costDTSP;
	private float curvature;
	private int numNodes;
	private bool visualizeBestTSP;
	private bool visualizeBestDTSP;
	private bool visualizeCurvature;

	// Awake is called when the script instance is being loaded, before any Start functions
	void Awake() {
		// Set up the reference
		text = GetComponent <TextMeshProUGUI> ();
	}

	void Update() {
		startAlgorithm = GameObject.Find("ButtonHandler").GetComponent<ButtonHandler>().startAlgorithm;
		stopAlgorithm = GameObject.Find("ButtonHandler").GetComponent<ButtonHandler>().stopAlgorithm;
		duration = GameObject.Find("ACO_DTSP").GetComponent<ACO_DTSP>().timer;
		costTSP = GameObject.Find("ACO_DTSP").GetComponent<ACO_DTSP>().bestTourDst;
		costDTSP = GameObject.Find("ACO_DTSP").GetComponent<ACO_DTSP>().costDubinsPath;
		isTSP = GameObject.Find("ACO_DTSP").GetComponent<ACO_DTSP>().isTSP;
		getFinalDubins = GameObject.Find("ACO_DTSP").GetComponent<ACO_DTSP>().getFinalDubins;
		curvature = GameObject.Find("ACO_DTSP").GetComponent<ACO_DTSP>().radiusOfCurvature;
		
		numNodes = (GameObject.Find("ACO_DTSP").GetComponent<ACO_DTSP>().numNodesPlaced < 3) ?
		GameObject.Find("ACO_DTSP").GetComponent<ACO_DTSP>().numNodes : 
		GameObject.Find("ACO_DTSP").GetComponent<ACO_DTSP>().numNodesPlaced;
		
		visualizeBestTSP = isTSP;
		visualizeBestDTSP = (!isTSP) || (isTSP && getFinalDubins && stopAlgorithm);
		visualizeCurvature = !isTSP || (isTSP && getFinalDubins);
		// Update displayed text
		text.SetText("Num of nodes:\t" + numNodes.ToString() + "\n" +
					"Duration (s):\t\t" + duration.ToString("0.00") + "\n" +
					"Best dst TSP:\t" + ((visualizeBestTSP) ? costTSP.ToString("0.000") : "---") + "\n" +
					"Best dst DTSP:\t" + ((visualizeBestDTSP) ? costDTSP.ToString("0.000") : "---") + "\n" +
					"Curvature:\t\t" + ((visualizeCurvature) ? curvature.ToString("0.00") : "None"));
	}
}
