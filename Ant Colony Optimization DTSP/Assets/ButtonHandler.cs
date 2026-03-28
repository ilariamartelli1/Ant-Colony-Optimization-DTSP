using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ButtonHandler : MonoBehaviour {
	
	[HideInInspector] public bool startAlgorithm = false;
	[HideInInspector] public bool stopAlgorithm = false;

	public void OnButtonPressStart() {
		startAlgorithm = true;
		stopAlgorithm = false;
	}

	public void OnButtonPressStop() {
		stopAlgorithm = true;
		startAlgorithm = false;
	}
}
