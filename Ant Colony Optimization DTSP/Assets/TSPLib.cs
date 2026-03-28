using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.Linq;

public class TSPLib : MonoBehaviour {

	public static T[] GetRow<T>(T[,] matrix, int rowNumber) {
		return Enumerable.Range(0, matrix.GetLength(1))
				.Select(x => matrix[rowNumber, x])
				.ToArray();
	}

	public static T[] GetColumn<T>(T[,] matrix, int columnNumber) {
		return Enumerable.Range(0, matrix.GetLength(0))
				.Select(x => matrix[x, columnNumber])
				.ToArray();
	}

	public static T[,] SetRow<T>(T[,] matrix, T[] row, int rowNumber) {
		for (int x = 0; x < matrix.GetLength(1); x++)
			matrix[rowNumber, x] = row[x];
		return matrix;
	}

	public static T[,] SetColumn<T>(T[,] matrix, T[] column, int columnNumber) {
		for (int x = 0; x < matrix.GetLength(0); x++)
			matrix[x, columnNumber] = column[x];
		return matrix;
	}

	public static T[] Shuffle<T>(T[] arr) {
		System.Random rnd = new System.Random();
		for (int i = arr.Length; i > 1; i--) {
			int j = rnd.Next(i);
			// Swaps value in cell i-1 with the one in cell j
			var val = arr[i - 1];
			arr[i - 1] = arr[j];
			arr[j] = val;
		}
		return arr;
	}

	public static (float, float, float) Points2Line(float x1, float y1, float x2, float y2) {
		// Returns a,b,c of the implicit form a*x + b*y + c = 0
		if (x1 == x2 && y1 == y2)	// "Error: same point"
			return (0,1,-y1);		// Leads back to the Orizontal case
		if (x1 == x2)				// Vertical case
			return (1,0,-x1);
		float m, q;
		m = (y1-y2)/(x1-x2);
		q = y1 - m * x1;
		return (-m,1,-q);
	}

	public static (float, float) PointOnPerpendicular(float af, float bf, float cf, float xf, float yf) {
		// Takes coefficients of a line and coordinates of a point
		// Returns the intersection to the perpendicular passing for the first point
		// Converts to double to avoid approssimation errors
		double a = (double)af, b = (double)bf, c = (double)cf, x = (double)xf, y = (double)yf;
		double aP,bP,cP,xP,yP;
		if (a == 0) // orizontal line
			(aP,bP,cP) = (1,0,-x); // perpendicular --> vertical line
		else
			(aP,bP,cP) = (-b/a, 1, (b/a)*x-y); // angular coefficient of the perpendicular line
		if (b == 0) { // vertical line
			xP = -c/a;
			yP = y;
		} else {
			xP = (bP*c/b - cP)/(aP - bP*a/b);
			yP = -(a*xP + c)/b;
		}
		float xPf = (float)xP, yPf = (float)yP;
		return (xPf,yPf);
	}

	public static float Points2Dist(float x1, float y1, float x2, float y2) {
		return Mathf.Sqrt(Mathf.Pow(x1 - x2, 2) + Mathf.Pow(y1 - y2, 2));
	}

	public (float[,], int) SetConfigurationCircle(float[,] posNodes, int numNodes) {
		// Approximate circumference, numNodes as the number of inscribed polygon vertices
		// Shortest ideal cycle length = 2*pi*50 = 314.159 (perfect circumference, numNodes --> Infinity)
		float theta = 2*Mathf.PI/numNodes, r = 50.0f;
		posNodes = new float[numNodes,2];
		for (int i = 0; i < numNodes; i++) {
			(posNodes[i,0], posNodes[i,1]) = (r*Mathf.Cos(i*theta), r*Mathf.Sin(i*theta));
		}
		return (posNodes, numNodes);
	}

	public (float[,], int) SetConfigurationStar(float[,] posNodes, int numNodes) {
		float theta = -(180-36)*Mathf.PI/180, r = 20.0f, vel_x = r, vel_y = 0;
		int count = 1;
		numNodes = 25;
		posNodes = new float[numNodes,2];
		(posNodes[0,0], posNodes[0,1]) = (0, 0); // first node
		for (int i = 1; i < numNodes; i++) {
			if (i % 5 == 0) {
				(vel_x, vel_y) = (r*Mathf.Cos(count*theta), r*Mathf.Sin(count*theta));
				count++;
			}
			(posNodes[i,0], posNodes[i,1]) = (posNodes[i-1,0] + vel_x, posNodes[i-1,1] + vel_y);
		}
		return (posNodes, numNodes);
	}

	public (float[,], int) SetConfigurationP01(float[,] posNodes, int numNodes) {
		// Shortest cycle length = 284.381 (found)
		// Shortest cycle length (if each distance is rounded to the nearest integer) = 291
		// Best path: 1 13 2 15 9 5 7 3 12 14 10 8 6 4 11
		numNodes = 15;
		posNodes = new float[numNodes,2];
		( posNodes[0,0],  posNodes[0,1]) = ( -0.0000000400893815f,   0.0000000358808126f);
		( posNodes[1,0],  posNodes[1,1]) = (-28.8732862244731230f,  -0.0000008724121069f);
		( posNodes[2,0],  posNodes[2,1]) = (-79.2915791686897506f,  21.4033307581457670f);
		( posNodes[3,0],  posNodes[3,1]) = (-14.6577381710829471f,  43.3895496964974043f);
		( posNodes[4,0],  posNodes[4,1]) = (-64.7472605264735108f, -21.8981713360336698f);
		( posNodes[5,0],  posNodes[5,1]) = (-29.0584693142401171f,  43.2167287683090606f);
		( posNodes[6,0],  posNodes[6,1]) = (-72.0785319657452987f,  -0.1815834632498404f);
		( posNodes[7,0],  posNodes[7,1]) = (-36.0366489745023770f,  21.6135482886620949f);
		( posNodes[8,0],  posNodes[8,1]) = (-50.4808382862985496f,  -7.3744722432402208f);
		( posNodes[9,0],  posNodes[9,1]) = (-50.5859026832315024f,  21.5881966132975371f);
		(posNodes[10,0], posNodes[10,1]) = ( -0.1358203773809326f,  28.7292896751977480f);
		(posNodes[11,0], posNodes[11,1]) = (-65.0865638413727368f,  36.0624693073746769f);
		(posNodes[12,0], posNodes[12,1]) = (-21.4983260706612533f,  -7.3194159498090388f);
		(posNodes[13,0], posNodes[13,1]) = (-57.5687244704708050f,  43.2505562436354225f);
		(posNodes[14,0], posNodes[14,1]) = (-43.0700258454450875f, -14.5548396888330487f);
		return (posNodes, numNodes);
	}

	public (float[,], int) SetConfigurationUlysses22(float[,] posNodes, int numNodes) {
		// Shortest cycle length = N/A
		// Shortest cycle length (if each distance is rounded to the nearest integer) = N/A (7013)
		// Best path: 1 14 13 12 7 6 15 5 11 9 10 19 20 21 16 3 2 17 22 4 18 8
		numNodes = 22;
		posNodes = new float[numNodes,2];
		( posNodes[0,0],  posNodes[0,1]) = (38.24f, 20.42f);
		( posNodes[1,0],  posNodes[1,1]) = (39.57f, 26.15f);
		( posNodes[2,0],  posNodes[2,1]) = (40.56f, 25.32f);
		( posNodes[3,0],  posNodes[3,1]) = (36.26f, 23.12f);
		( posNodes[4,0],  posNodes[4,1]) = (33.48f, 10.54f);
		( posNodes[5,0],  posNodes[5,1]) = (37.56f, 12.19f);
		( posNodes[6,0],  posNodes[6,1]) = (38.42f, 13.11f);
		( posNodes[7,0],  posNodes[7,1]) = (37.52f, 20.44f);
		( posNodes[8,0],  posNodes[8,1]) = (41.23f,  9.10f);
		( posNodes[9,0],  posNodes[9,1]) = (41.17f, 13.05f);
		(posNodes[10,0], posNodes[10,1]) = (36.08f, -5.21f);
		(posNodes[11,0], posNodes[11,1]) = (38.47f, 15.13f);
		(posNodes[12,0], posNodes[12,1]) = (38.15f, 15.35f);
		(posNodes[13,0], posNodes[13,1]) = (37.51f, 15.17f);
		(posNodes[14,0], posNodes[14,1]) = (35.49f, 14.32f);
		(posNodes[15,0], posNodes[15,1]) = (39.36f, 19.56f);
		(posNodes[16,0], posNodes[16,1]) = (38.09f, 24.36f);
		(posNodes[17,0], posNodes[17,1]) = (36.09f, 23.00f);
		(posNodes[18,0], posNodes[18,1]) = (40.44f, 13.57f);
		(posNodes[19,0], posNodes[19,1]) = (40.33f, 14.15f);
		(posNodes[20,0], posNodes[20,1]) = (40.37f, 14.23f);
		(posNodes[21,0], posNodes[21,1]) = (37.57f, 22.56f);
		return (posNodes, numNodes);
	}

	public (float[,], int) SetConfigurationOliver30(float[,] posNodes, int numNodes) {
		// Shortest cycle length = 423.741
		// Shortest cycle length (if each distance is rounded to the nearest integer) = 420
		// Best path: 1 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 25 24 26 27 28 29 30 2
		numNodes = 30;
		posNodes = new float[numNodes,2];
		( posNodes[0,0],  posNodes[0,1]) = (54, 67);
		( posNodes[1,0],  posNodes[1,1]) = (54, 62);
		( posNodes[2,0],  posNodes[2,1]) = (37, 84);
		( posNodes[3,0],  posNodes[3,1]) = (41, 94);
		( posNodes[4,0],  posNodes[4,1]) = ( 2, 99);
		( posNodes[5,0],  posNodes[5,1]) = ( 7, 64);
		( posNodes[6,0],  posNodes[6,1]) = (25, 62);
		( posNodes[7,0],  posNodes[7,1]) = (22, 60);
		( posNodes[8,0],  posNodes[8,1]) = (18, 54);
		( posNodes[9,0],  posNodes[9,1]) = ( 4, 50);
		(posNodes[10,0], posNodes[10,1]) = (13, 40);
		(posNodes[11,0], posNodes[11,1]) = (18, 40);
		(posNodes[12,0], posNodes[12,1]) = (24, 42);
		(posNodes[13,0], posNodes[13,1]) = (25, 38);
		(posNodes[14,0], posNodes[14,1]) = (44, 35);
		(posNodes[15,0], posNodes[15,1]) = (41, 26);
		(posNodes[16,0], posNodes[16,1]) = (45, 21);
		(posNodes[17,0], posNodes[17,1]) = (58, 35);
		(posNodes[18,0], posNodes[18,1]) = (62, 32);
		(posNodes[19,0], posNodes[19,1]) = (82,  7);
		(posNodes[20,0], posNodes[20,1]) = (91, 38);
		(posNodes[21,0], posNodes[21,1]) = (83, 46);
		(posNodes[22,0], posNodes[22,1]) = (71, 44);
		(posNodes[23,0], posNodes[23,1]) = (64, 60);
		(posNodes[24,0], posNodes[24,1]) = (68, 58);
		(posNodes[25,0], posNodes[25,1]) = (83, 69);
		(posNodes[26,0], posNodes[26,1]) = (87, 76);
		(posNodes[27,0], posNodes[27,1]) = (74, 78);
		(posNodes[28,0], posNodes[28,1]) = (71, 71);
		(posNodes[29,0], posNodes[29,1]) = (58, 69);
		return (posNodes, numNodes);
	}

	public (float[,], int) SetConfigurationDantzig42(float[,] posNodes, int numNodes) {
		// Shortest cycle length = 682.752 (found)
		// Shortest cycle length (if each distance is rounded to the nearest integer) = 699
		// Best path: -
		numNodes = 42;
		posNodes = new float[numNodes,2];
		( posNodes[0,0],  posNodes[0,1]) = (   170,  85);
		( posNodes[1,0],  posNodes[1,1]) = (   166,  88);
		( posNodes[2,0],  posNodes[2,1]) = (   133,  73);
		( posNodes[3,0],  posNodes[3,1]) = (   140,  70);
		( posNodes[4,0],  posNodes[4,1]) = (   142,  55);
		( posNodes[5,0],  posNodes[5,1]) = (   126,  53);
		( posNodes[6,0],  posNodes[6,1]) = (   125,  60);
		( posNodes[7,0],  posNodes[7,1]) = (   119,  68);
		( posNodes[8,0],  posNodes[8,1]) = (   117,  74);
		( posNodes[9,0],  posNodes[9,1]) = (    99,  83);
		(posNodes[10,0], posNodes[10,1]) = (    73,  79);
		(posNodes[11,0], posNodes[11,1]) = (    72,  91);
		(posNodes[12,0], posNodes[12,1]) = (    37,  94);
		(posNodes[13,0], posNodes[13,1]) = (     6, 106);
		(posNodes[14,0], posNodes[14,1]) = (     3,  97);
		(posNodes[15,0], posNodes[15,1]) = (    21,  82);
		(posNodes[16,0], posNodes[16,1]) = (    33,  67);
		(posNodes[17,0], posNodes[17,1]) = (     4,  66);
		(posNodes[18,0], posNodes[18,1]) = (     3,  42);
		(posNodes[19,0], posNodes[19,1]) = (    27,  33);
		(posNodes[20,0], posNodes[20,1]) = (    52,  41);
		(posNodes[21,0], posNodes[21,1]) = (    57,  59);
		(posNodes[22,0], posNodes[22,1]) = (    58,  66);
		(posNodes[23,0], posNodes[23,1]) = (    88,  65);
		(posNodes[24,0], posNodes[24,1]) = (    99,  67);
		(posNodes[25,0], posNodes[25,1]) = (    95,  55);
		(posNodes[26,0], posNodes[26,1]) = (    89,  55);
		(posNodes[27,0], posNodes[27,1]) = (    83,  38);
		(posNodes[28,0], posNodes[28,1]) = (    85,  25);
		(posNodes[29,0], posNodes[29,1]) = (   104,  35);
		(posNodes[30,0], posNodes[30,1]) = (   112,  37);
		(posNodes[31,0], posNodes[31,1]) = (   112,  24);
		(posNodes[32,0], posNodes[32,1]) = (   113,  13);
		(posNodes[33,0], posNodes[33,1]) = (   125,  30);
		(posNodes[34,0], posNodes[34,1]) = (   135,  32);
		(posNodes[35,0], posNodes[35,1]) = (   147,  18);
		(posNodes[36,0], posNodes[36,1]) = (147.5f,  36);
		(posNodes[37,0], posNodes[37,1]) = (154.5f,  45);
		(posNodes[38,0], posNodes[38,1]) = (   157,  54);
		(posNodes[39,0], posNodes[39,1]) = (   158,  61);
		(posNodes[40,0], posNodes[40,1]) = (   172,  82);
		(posNodes[41,0], posNodes[41,1]) = (   174,  87);
		return (posNodes, numNodes);
	}

	public (float[,], int) SetConfigurationAtt48(float[,] posNodes, int numNodes) {
		// Shortest cycle length =  (found)
		// Shortest cycle length (if each distance is rounded to the nearest integer) = 6850 (?)
		// Best path: 1 8 38 31 44 18 7 28 6 37 19 27 17 43 30 36 46 33 20 47 21 32 39 48 5 42 24 10 45 35 4 26 2 29 34 41 16 22 3 23 14 25 13 11 12 15 40 9
		numNodes = 48;
		posNodes = new float[numNodes,2];
		( posNodes[0,0],  posNodes[0,1]) = (6734, 1453);
		( posNodes[1,0],  posNodes[1,1]) = (2233,   10);
		( posNodes[2,0],  posNodes[2,1]) = (5530, 1424);
		( posNodes[3,0],  posNodes[3,1]) = ( 401,  841);
		( posNodes[4,0],  posNodes[4,1]) = (3082, 1644);
		( posNodes[5,0],  posNodes[5,1]) = (7608, 4458);
		( posNodes[6,0],  posNodes[6,1]) = (7573, 3716);
		( posNodes[7,0],  posNodes[7,1]) = (7265, 1268);
		( posNodes[8,0],  posNodes[8,1]) = (6898, 1885);
		( posNodes[9,0],  posNodes[9,1]) = (1112, 2049);
		(posNodes[10,0], posNodes[10,1]) = (5468, 2606);
		(posNodes[11,0], posNodes[11,1]) = (5989, 2873);
		(posNodes[12,0], posNodes[12,1]) = (4706, 2674);
		(posNodes[13,0], posNodes[13,1]) = (4612, 2035);
		(posNodes[14,0], posNodes[14,1]) = (6347, 2683);
		(posNodes[15,0], posNodes[15,1]) = (6107,  669);
		(posNodes[16,0], posNodes[16,1]) = (7611, 5184);
		(posNodes[17,0], posNodes[17,1]) = (7462, 3590);
		(posNodes[18,0], posNodes[18,1]) = (7732, 4723);
		(posNodes[19,0], posNodes[19,1]) = (5900, 3561);
		(posNodes[20,0], posNodes[20,1]) = (4483, 3369);
		(posNodes[21,0], posNodes[21,1]) = (6101, 1110);
		(posNodes[22,0], posNodes[22,1]) = (5199, 2182);
		(posNodes[23,0], posNodes[23,1]) = (1633, 2809);
		(posNodes[24,0], posNodes[24,1]) = (4307, 2322);
		(posNodes[25,0], posNodes[25,1]) = ( 675, 1006);
		(posNodes[26,0], posNodes[26,1]) = (7555, 4819);
		(posNodes[27,0], posNodes[27,1]) = (7541, 3981);
		(posNodes[28,0], posNodes[28,1]) = (3177,  756);
		(posNodes[29,0], posNodes[29,1]) = (7352, 4506);
		(posNodes[30,0], posNodes[30,1]) = (7545, 2801);
		(posNodes[31,0], posNodes[31,1]) = (3245, 3305);
		(posNodes[32,0], posNodes[32,1]) = (6426, 3173);
		(posNodes[33,0], posNodes[33,1]) = (4608, 1198);
		(posNodes[34,0], posNodes[34,1]) = (  23, 2216);
		(posNodes[35,0], posNodes[35,1]) = (7248, 3779);
		(posNodes[36,0], posNodes[36,1]) = (7762, 4595);
		(posNodes[37,0], posNodes[37,1]) = (7392, 2244);
		(posNodes[38,0], posNodes[38,1]) = (3484, 2829);
		(posNodes[39,0], posNodes[39,1]) = (6271, 2135);
		(posNodes[40,0], posNodes[40,1]) = (4985,  140);
		(posNodes[41,0], posNodes[41,1]) = (1916, 1569);
		(posNodes[42,0], posNodes[42,1]) = (7280, 4899);
		(posNodes[43,0], posNodes[43,1]) = (7509, 3239);
		(posNodes[44,0], posNodes[44,1]) = (  10, 2676);
		(posNodes[45,0], posNodes[45,1]) = (6807, 2993);
		(posNodes[46,0], posNodes[46,1]) = (5185, 3258);
		(posNodes[47,0], posNodes[47,1]) = (3023, 1942);
		return (posNodes, numNodes);
	}
}
