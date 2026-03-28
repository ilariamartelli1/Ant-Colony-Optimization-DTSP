using UnityEngine;
using UnityEngine.UI;
using System;
using TMPro;

public class TextManager : MonoBehaviour {

	public TextMeshProUGUI text;

	private bool startStat;
	private bool? isTSP;
	private bool? getFinalDubins;

	private float? duration;
	private float? costTSP;
	private float? costDTSP;

	private float? curvature;
	private int? numNodes;
	private int? numIterations;
	private int? currRepetition;

	private bool visualizeBestTSP;
	private bool visualizeBestDTSP;
	private bool visualizeCurvature;
	private bool visualizeRep;

	void Awake() {
		// Set up the reference
		text = GetComponent <TextMeshProUGUI> ();
	}

	void Update() {
		startStat = GameObject.Find("ButtonHandler").GetComponent<ButtonHandler>().startAlgorithm;

		duration = GameObject.Find("Wrapper").GetComponent<ACODTSP_Wrapper>()?.timer;
		costTSP = GameObject.Find("Wrapper").GetComponent<ACODTSP_Wrapper>()?.bestTourDst;
		costDTSP = GameObject.Find("Wrapper").GetComponent<ACODTSP_Wrapper>()?.costDubinsPath;
		isTSP = GameObject.Find("Wrapper").GetComponent<ACODTSP_Wrapper>()?.isTSP;
		getFinalDubins = GameObject.Find("Wrapper").GetComponent<ACODTSP_Wrapper>()?.getFinalDubins;
		curvature = GameObject.Find("Wrapper").GetComponent<ACODTSP_Wrapper>()?.radiusOfCurvature;
		numNodes = GameObject.Find("Wrapper").GetComponent<ACODTSP_Wrapper>()?.numNodesPlaced;
		numIterations = GameObject.Find("Wrapper").GetComponent<ACODTSP_Wrapper>()?.numIterations;
		currRepetition = GameObject.Find("Wrapper").GetComponent<Wrapper>()?.countStat;
		
		visualizeBestTSP = (bool)isTSP && startStat;
		visualizeBestDTSP = (!(bool)isTSP && startStat) || ((bool)isTSP && (bool)getFinalDubins);
		visualizeCurvature = !(bool)isTSP || ((bool)isTSP && (bool)getFinalDubins);
		visualizeRep = currRepetition != 0;

		// if (numNodes.HasValue && duration.HasValue && costTSP.HasValue && costDTSP.HasValue && curvature.HasValue)
		// Update displayed text
		text.SetText("Num of nodes:\t" + numNodes.Value.ToString() + "\n" +
					"Duration (s):\t\t" + duration.Value.ToString("0.00") + "\n" +
					"Best dst TSP:\t" + ((visualizeBestTSP) ? costTSP.Value.ToString("0.000") : "---") + "\n" +
					"Best dst DTSP:\t" + ((visualizeBestDTSP) ? costDTSP.Value.ToString("0.000") : "---") + "\n" +
					"Curvature:\t\t" + ((visualizeCurvature) ? curvature.Value.ToString("0.00") : "None") + "\n" +
					"Max iterations:\t" + numIterations.Value.ToString() + "\n" +
					"Current rep:\t\t" + ((visualizeRep) ? currRepetition.Value.ToString() : "---"));
	}
}
