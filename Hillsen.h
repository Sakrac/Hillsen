#pragma once

#include <inttypes.h>
#include <vector>

struct Pos2 {
	int x, y;
	Pos2(int _x, int _y) : x(_x), y(_y) {}
};

typedef std::vector<struct Pos2> Pos2List;

struct Hill {
	const int w, d;
	bool stop;
	double *hgt;

	Hill(int width, int depth) : w(width), d(depth), stop(false) {}
	~Hill() { free(hgt); }

	// center x, y; margin; height
	void SimpleHill(double cx, double cy, double m, double h);
	void DomeHill(double h, double lin);

	// Dent the target map at the position
	void Dent(double cx, double cy, double r, double zx, double zy);
	void RandomDent(double max_part, double max_cut, double rmin, double rmax);

	void Crack(double cx, double cy, double r, double ang, double h);
	void RandomCrack(double rmin, double rmax, double hmin, double hmax);

	void FindHoles(int radius, double dir, Pos2List &holes);

	// soften the hill
	void RadialAvg(int rad, double scale);
	void GaussianBlur(double sigma, double scale);

	// write a png file to preview the height map
	void WriteHgtMap(const char *filename);

	// rgb isometric representation
	uint8_t* IsometricView(int &width, int &height);

	struct Normal {
		double x, y, z;
	};
	Normal GetNormalAtPoint(int x, int y);

	// rgb perspective, plus optional depth map
//	uint8_t* VoxelView(uint16_t **depth, int width, int height, double cam_offs, double cam_side, double tan_fov, double ang, double pitch);
	uint8_t* VoxelView(uint8_t *pImg, int width, int height, double cam_offs, double cam_side, double cam_up, double tan_fov, double ang, double pitch);
	Pos2 ScreenProj(int width, int height, int x, int y, double cam_offs, double cam_side, double cam_up, double tan_fov, double ang, double pitch);
	Pos2 ScreenHit(int width, int height, int x, int y, double cam_offs, double cam_side, double cam_up, double tan_fov, double ang, double pitch);
};

Hill* CreateMap(int width, int depth);

