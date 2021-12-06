//
//  main.cpp
//  Hillsen
//
//  Created by Carl-Henrik Skårstedt on 2/2/16.
//  Copyright © 2016 Carl-Henrik Skårstedt. All rights reserved.
//


#include "stdafx.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stdlib.h>
#include "stb_image_write.h"
#include <inttypes.h>
#include <math.h>
#include <time.h>
#include <vector>
#include "hillsen.h"

#define clamp(x,mn,mx) ((x)<(mn)?(mn):((x)>(mx)?(mx):(x)))

// make a hill with planar 0 height
Hill* CreateMap(int width, int depth)
{
	Hill *hill = new Hill(width, depth);
	hill->hgt = (double*)calloc(1, sizeof(double) * width * depth);
	
	return hill;
}

void Hill::SimpleHill(double cx, double cy, double m, double h)
{
	double lx = h/(cx-m);
	double hx = h/(1.0-m-cx);
	double ly = h/(cy-m);
	double hy = h/(1.0-m-cy);
	
	for (int y=0; y<d; y++) {
		double *trg = hgt + y * w;
		double dy = (double)y/(double)d;
		if (dy>m && dy<(1-m)) {
			double vy = dy<cy ? (dy-m)*ly : (1-m-dy)*hy;
			for (int x=0; x<w; x++) {
				double dx = (double)x/(double)w;
				if (dx>m && dx<(1-m)) {
					double vx = dx<cx ? (dx-m)*lx : (1-m-dx)*hx;
					*trg++ = vy<vx ? vy : vx;
				} else
					++trg;
			}
		}
	}
}

void Hill::DomeHill(double h, double lin)
{
	double r = sqrt((double)(d*d+w*w));
	int cx = w/2;
	int cy = d/2;
	double *trg = hgt;
	for (int y = 0; y<d; y++) {
		for (int x = 0; x<w; x++) {
			double dist = sqrt(double((x-cx)*(x-cx)+(y-cy)*(y-cy)));
			double ar = sin(1.0-(dist/r)*2.0) * (1.0-lin) + dist/r * lin;
			*trg++ = h * ar;
		}
	}
}

void Hill::Dent(double cx, double cy, double r, double zx, double zy)
{
	// find a vertical range
	int y0 = (int)(cy-r);
	int y1 = (int)(cy+r);
	if (y0<0)
		y0 = 0;
	if (y1>=d)
		y1 = d-1;
	for (int y=y0; y<=y1; y++) {
		double oy = (double)y-cy;
		if (oy<r) {
			double vy = oy * zy;
			double xr = sqrt(r*r-oy*oy);
			int x0 = (int)(cx-xr);
			int x1 = (int)(cx+xr);
			if (x0<0)
				x0 = 0;
			if (x1>=w)
				x1 = w-1;
			double ox = (double)x0 - cx;
			double vz = vy + ox * zx;
			double *trg = hgt + y * w + x0;
			for (int x=x0; x<=x1; x++) {
				*trg++ += vz;
				vz += zx;
			}
		}
	}
}
#define square(a) ((a)*(a))
void Hill::Crack(double cx, double cy, double r, double ang, double h)
{
	int cxi = (int)cx;
	int cyi = (int)cy;
	int y0 = (int)(cy-r);
	int y1 = (int)(cy+r);
	double nx = cos(ang);
	double ny = sin(ang);
	double piir = M_PI/square(r);
	if (y0<0)
		y0 = 0;
	if (y1>=d)
		y1 = d-1;
	double dy = (double)y0-cy;
	for (int y = y0; y<=y1; y++) {
		double oy = (double)y-cy;
		if (oy<r) {
			double xr = sqrt(square(r) - square(oy));
			int x0 = (int)(cx-xr-0.75);
			int x1 = (int)(cx+xr+0.75);
			if (x0<0)
				x0 = 0;
			if (x1>=w)
				x1 = w-1;
			double ox = (double)x0 - cx;
			double *trg = hgt + y * w + x0;
			double ndy = -dy*ny;
			double dx = (double)x0-cx;
			double dy2 = square(dy);
			for (int x = x0; x<=x1; x++) {
				double o = square(dx) + dy2;
				double d2 = piir * (square(dx)+square(dy));
				if (d2 > M_PI_2)
					d2 = M_PI - d2;
				double a = (0.5-0.04166666666666666666666666666667*d2)*d2;

				//1.2337005501361698273543113749845
				//0.25366950790104801363656336637684
				*trg++ += (dx*nx>ndy ? a : -a)*h;
				dx += 1.0;
			}
		}
		dy += 1.0;
	}

}


void Hill::FindHoles(int radius, double dir, Pos2List &holes)
{
	// exclude edges
	int margin = radius;
	int ew = w-margin;
	int ed = d-margin;
	int ln = w;

	const double *m = hgt + (margin * w  + margin);
	for (int y = margin; y<ed; ++y) {
		for (int x = margin; x<ew; ++x) {
			bool b = true;
			double v = *m;
			for (int l = 1-radius; b && l<radius; l++) {
				const double *r = m+l*ln;
				int rw = (int)sqrt(radius*radius-l*l);
				for (int s = 1-rw; s<rw; s++) {
					if ((s || l) && (r[s]-v)*dir <= 0.0) {
						b = false;
						break;
					}
				}
			}
			if (b)
				holes.push_back(struct Pos2(x, y));
			++m;
		}
		m += 2 * margin;
	}
}
//#pragma optimize("", off)
void Hill::GaussianBlur(double sigma, double scale)
{
	int radius = (int)(3.0 * sigma+0.5);
	double *values = new double[radius*3+2+64];
	double *cache = values + radius;
	double div = 1.0/sqrt(2.0*M_PI)/sigma;
	for (int r = 0; r<radius; r++)
		values[r] = exp(-r*r/(2.0*sigma*sigma)) * div;
	int pts = radius * 2 + 1;
	int beg = 0;
	// left->right
//	double *real_hgt = hgt;
//	double *buf = (double*)malloc(sizeof(double) * w * d);
//	memcpy(buf, hgt, sizeof(double) * w * d);
	double *write = hgt;// buf;
	const double *read = hgt;
//	hgt = buf;
	int end_w = w-radius-1;
	int last = pts-1;
	for (int y = 0; y<d && !stop; y++) {
		double v = *read;
		beg = 0;
		for (int r = 0; r<radius; r++) {
			cache[radius-1-r] = 2.0*v - *read; // left
//			cache[r] = v;	// left
			cache[r+radius+1] = *read++; // right
		}
		cache[radius] = v; // center
		for (int x = 0; x<w; x++) {
			double b = 0.0;
			for (int p = 1-radius; p<radius; p++)
				b += values[abs(p)] * cache[(p+radius-1 + beg) % pts];
			*write++ = b;
			if (x<end_w)
				++read;
			cache[(last + beg++) % pts] = *read; // right side
		}
		++read;
	}
//	hgt = real_hgt;
	int end_h = d-radius-1;
	write = hgt;
	read = hgt;
	for (int x = 0; x<w && !stop; x++) {
		const double *ready = read;
		double v = *read++;
		beg = 0;
		for (int r = 0; r<radius; r++) {
			cache[radius-1-r] = 2.0*v - *ready; // left
//			cache[r] = v;
			cache[r+radius+1] = *ready;
			ready += w;
		}
		cache[radius] = v;
		for (int y = 0; y<d; y++) {
			double b = 0.0;
			for (int p = 1-radius; p<radius; p++)
				b += values[abs(p)] * cache[(p+radius-1 + beg) % pts];
			write[x+y*w] = b;
			if (y<end_h)
				ready += w;
			cache[(last + beg++) % pts] = *ready;
		}
	}

//	free(buf);
	delete []values;
}
//#pragma optimize("", on)


void Hill::RadialAvg(int rad, double scale)
{
	double *buf = (double*)calloc(1, sizeof(double) * w * d);

	double *wrt = buf, *read = hgt;
	for (int y = 0; y<d; ++y) {
		int y0 = y>rad ? -rad : -y;
		int y1 = (d-y)>rad ? rad : (d-y-1);
		for (int x = 0; x<w; ++x) {
			int x0 = x>rad ? -rad : -x;
			int x1 = (w-x)>rad ? rad : (w-x-1);
			int n = 0;
			double s = 0.0;
			for (int ry = y0; ry<=y1; ry++) {
				for (int rx = x0; rx<=x1; rx++) {
					if ((rx*rx+ry*ry)<=(rad*rad)) {
						n++;
						s += hgt[(y+ry) * w + x+rx];
					}
				}
			}
			if (n)
				*wrt++ = (*read++) * (1.0-scale) + scale * s / (double)n;
			else
				*wrt++ = *read++;
		}
	}
	memcpy(hgt, buf, w*d*sizeof(double));
	free(buf);
}

void Hill::RandomDent(double max_part, double max_cut, double rmin, double rmax)
{
	double ang = (1.0 - 2.0 *(double)rand() / (double)RAND_MAX)*M_PI;
	double radius = (double)rand()/(double)RAND_MAX;
	
	radius = (rmax-rmin) * radius + rmin;

	double max_tilt = radius * max_part;
	if (max_tilt>max_cut)
		max_tilt = max_cut;
	double tilt = (max_tilt/radius) * (double)rand() / (double)RAND_MAX;
	
	double zx = cos(ang) * tilt;
	double zy = sin(ang) * tilt;
	
	double cx = w * (double)rand()/(double)RAND_MAX;
	double cy = d * (double)rand()/(double)RAND_MAX;
	Dent(cx, cy, radius, zx, zy);
}

void Hill::RandomCrack(double rmin, double rmax, double hmin, double hmax)
{
	double ang = (1.0 - 2.0 *(double)rand() / (double)RAND_MAX)*M_PI;
	double radius = (rmax-rmin)*(double)rand()/(double)RAND_MAX + rmin;
	double hgt = (hmax-hmin)*(double)rand()/(double)RAND_MAX + hmin;
	double cx = w * (double)rand()/(double)RAND_MAX;
	double cy = d * (double)rand()/(double)RAND_MAX;
	Crack(cx, cy, radius, ang, hgt);
}


void Hill::WriteHgtMap(const char *filename)
{
	double mn = 1e64;
	double mx = -mn;
	
	double *read = hgt;
	for (int i=w*d; i; --i) {
		double v = *read++;
		mn = v<mn ? v:mn;
		mx = v>mx ? v:mx;
	}
	
	double add = -mn;
	double mul = mx>(mn+1e-5) ? (255.0 / (mx-mn)) : 0.0;
	
	uint8_t *map = (uint8_t*)malloc(w*d*sizeof(uint8_t));
	if (map) {
		uint8_t *wrt = map;
		read = hgt;
		for (int i = w*d; i; --i)
			*wrt++ = (uint8_t)((*read+++add) * mul);

		stbi_write_png(filename, w, d, 1, map, 0);

		free(map);
	}
}

struct Hill::Normal Hill::GetNormalAtPoint(int x, int y)
{
	const double *ctr = hgt + (x + y * w);
	double prev_h = x ? ctr[-1] : ctr[0];
	double next_h = (x+1)<w ? ctr[1] : ctr[0];
	double prev_v = y ? ctr[-w] : ctr[0];
	double next_v = (y+1)<d ? ctr[w] : ctr[0];

	// center height is C
	// neighbour heights are A, B, D, E
	//
	//       A
	//       | \
	//    D__C__B
	//     \ |
	//       E
	//
	// tri1: (0,0,C)-(0,1,A)-(1,0,B)
	// tri2: (0,0,C)-(0,-1,E)-(-1,0,D)
	// tri1 e1: (0-0, 1-0, A-C) = (0,1,A-C)
	// tri1 e2: (1-0, 0-1, B-A) = (1,-1,B-A)
	// tri2 e1: (0-0, -1-0, E-C) = (0,-1,E-C)
	// tri2 e2: (-1-9,0-1,D-E) = (-1,1,D-E)
	// tri1 e1xe2: (B-A)+(A-C), (A-C), -1 = B-C, A-C, -1
	// tri2 e1xe2: -(D-E)-(E-C), -(E-C), -1 = C-D, C-E, -1
	// Normal 1 = (C-E, C-D, 1)
	// Normal 2 = (A-C, B-C, 1)
	// average and normalize
	// invert winding order because up is +Z
	double Anx = next_h - prev_h;
	double Any = next_v - prev_v;
	double Anz = 2.0;
	double div = 1.0 / sqrt(Anx*Anx+Any*Any+Anz*Anz);
	Anx *= div;
	Any *= div;
	Anz *= div;
	struct Normal ret;
	ret.x = Anx;
	ret.y = Any;
	ret.z = Anz;
	return ret;
}

void ProjEdge(double sa, double ca, double sp, double cp, int width, int height, double cam_x, double cam_y, double cam_z, double fov)
{
	// find the projection of the bottom of the screen on the ground, then the scalar for along the ground
	const double side = 0.5 * width;
	const double dist = 0.5 * width / fov;
	const double hgt = 0.5 * height;
	const double dir_x = side * ca + dist * sa * cp - hgt * -sp * sa;
	const double dir_y = -side * sa + dist * ca * cp - hgt * -sp * ca;
	const double dir_z = dist * -sp - hgt * cp;

	// project ray on ground
	const double cam_gnd_x = cam_x - cam_z * dir_x / dir_z;
	const double cam_gnd_y = cam_y - cam_z * dir_y / dir_z;

	// side = 0, hgt = 0.5 * height
	const double cam_gnd_x0 = cam_x + cam_z * (width / fov * sa * cp + height * sp * sa) / (width / fov * sp + height * cp);
	const double cam_gnd_y0 = cam_y + cam_z * (width / fov * ca * cp + height * sp * ca) / (width / fov * sp + height * cp);

	// side = 0.5 * width, hgt = 0.5 * height
	const double cam_gnd_x1 = cam_x + cam_z * (width * ca + width / fov * sa * cp + height * sp * sa) / (width / fov * sp + height * cp);
	const double cam_gnd_y1 = cam_y + cam_z * (-width * sa + width / fov * ca * cp + height * sp * ca) / (width / fov * sp + height * cp);

	// bottom of the screen projected onto the z=0 plane center to right side
	const double half_scrn_x = cam_z * (width * ca) / (width / fov * sp + height * cp);
	const double half_scrn_y = cam_z * (-width * sa) / (width / fov * sp + height * cp);


	// side = 0, hgt = 0.5 * height - 1

	const double cam_gnd_x2 = cam_x + cam_z * (width / fov * sa * cp + (height-1) * sp * sa) / (width / fov * sp + (height-1) * cp);
	const double cam_gnd_y2 = cam_y + cam_z * (width / fov * ca * cp + (height-1) * sp * ca) / (width / fov * sp + (height-1) * cp);

	// side = 0.5 * width, hgt = 0.5 * height - 1

	const double cam_gnd_x3 = cam_x + cam_z * (width * ca + width / fov * sa * cp + (height-1) * sp * sa) / (width / fov * sp + (height-1) * cp);
	const double cam_gnd_y3 = cam_y + cam_z * (-width * sa + width / fov * ca * cp + (height-1) * sp * ca) / (width / fov * sp + (height-1) * cp);

	const double half_scrn_x1 = cam_z * (width * ca) / (width / fov * sp + (height-1) * cp);
	const double half_scrn_y1 = cam_z * (-width * sa) / (width / fov * sp + (height-1) * cp);

	const double step_fwd_x = cam_x + cam_z * (width / fov * sa * cp + (height-1) * sp * sa) / (width / fov * sp + (height-1) * cp) - (cam_x + cam_z * (width / fov * sa * cp + height * sp * sa) / (width / fov * sp + height * cp));
	const double step_fwd_y = cam_z * ((width / fov * ca * cp + (height-1) * sp * ca) / (width / fov * sp + (height-1) * cp) - (width / fov * ca * cp + height * sp * ca) / (width / fov * sp + height * cp));

	const double edge_x = half_scrn_x1 + cam_gnd_x2 - half_scrn_x - cam_gnd_x0;
	const double edge_y = half_scrn_y1 + cam_gnd_y2 - half_scrn_y - cam_gnd_y0;

	/*
	
Given two lines, L0 and L1
L0 = P0 + s D0
L1 = P1 +  t D1

Find a point along both lines s and t where L0 and L1 intersects.

P0 + s D0 = P1 + t D1

Replacing P0 and P1 with Pd = P1 - P0 and solving this for x and y gives:

denominator = D0y D1x - D0x D1y
s = (Pdy D1x - Pdx D1y) / denominator
t = (Pdx D0y - Pdy D0x) / denominator

	*/

	// Pd = half_scrn
	// 

	// intersection of lines to find ground focus point
//	const double focus_x

}

struct VoxEdge {
	int x0, y0, x1, y1;
};

static struct VoxEdge GetVoxelEdge(double fwd_x, double fwd_y, double fwd_z, double fwd_u, double side, double hgt, double dist, double cam_x, double cam_y, double cam_z, int w, int d)
{
	const double dir_x = side * fwd_y + dist * fwd_x * fwd_u - hgt * fwd_z * fwd_x;
	const double dir_y = -side * fwd_x + dist * fwd_y * fwd_u - hgt * fwd_z * fwd_y;
	const double dir_z = dist * fwd_z - hgt * fwd_u;

	// project ray on ground
	const double cam_gnd_x = cam_x - cam_z * dir_x / dir_z;
	const double cam_gnd_y = cam_y - cam_z * dir_y / dir_z;

	// project the next pixel up on ground
	const double d2d_x = cam_x - cam_z * (dir_x + fwd_x*fwd_z) / (dir_z + fwd_u) - cam_gnd_x;
	const double d2d_y = cam_y - cam_z * (dir_y + fwd_y*fwd_y) / (dir_z + fwd_y) - cam_gnd_y;

	const double rx0 = cam_gnd_x + d2d_x * (-cam_y)/d2d_y;
	const double rx1 = cam_gnd_x + d2d_x * (d-cam_y)/d2d_y;
	const double ry0 = cam_gnd_y + d2d_y * (-cam_x)/d2d_x;
	const double ry1 = cam_gnd_y + d2d_y * (w-cam_x)/d2d_x;

	int lx0 = -1, ly0 = -1, lx1 = -1, ly1 = -1;
	double t0 = -1.0;
	double t1 = -1.0;

	// get a line intersecting the box of the world
	if (fabs(dir_x)>0.00001) {
		double t = -cam_x/dir_x;
		double y0 = cam_y + dir_y * t;
		if (y0>=0.0 && y0<(double)d) {
			t0 = t;
			lx0 = 0;
			ly0 = (int)y0;
		}

		t = ((double)w-cam_x)/dir_x, 0.0;
		y0 = cam_y + dir_y * t;
		if (y0>=0.0 && y0<(double)d) {
			lx1 = w-1;
			ly1 = (int)y0;
			if (lx0<0 || t<t0) {
				t1 = t0; t0 = t;
				lx0 ^= lx1; lx1 ^= lx0; lx0 ^= lx1;
				ly0 ^= ly1; ly1 ^= ly0; ly0 ^= ly1;
			} else
				t1 = t;
		}
	}

	if (fabs(dir_y)>=0.00001 && lx1<0) {

		double t = -cam_y/dir_y;
		double x0 = cam_x + dir_x * t;
		if (x0>=0.0 && x0<(double)w) {
			int nx = (int)x0;
			int ny = 0;
			if (lx0<0 || t<t0) {
				t1 = t0; lx1 = lx0; ly1 = ly0;
				t0 = t; lx0 = nx; ly0 = ny;
			} else if (lx1<0 || t<t1) {
				t1 = t; lx1 = nx; ly1 = ny;
			}
		}
		t = (double(d)-cam_y)/dir_y;
		x0 = cam_x + dir_x * t;
		if (x0>=0.0 && x0<(double)w) {
			if (t<t0) {
				t1 = t0; lx1 = lx0; ly1 = ly0;
				t0 = t;
				lx0 = (int)x0;
				ly0 = d-1;
			} else if (t>t1) {
				t1 = t;
				lx1 = (int)x0;
				ly1 = d-1;
			}
		}
	}
	struct VoxEdge e;
	e.x0 = lx0;
	e.y0 = ly0;
	e.x1 = lx1;
	e.y1 = ly1;
	return e;
}
//#pragma optimize("", off)
int hov_x = -1000, hov_y = -1000;
// find a voxel map position from a screen position
Pos2 Hill::ScreenHit(int width, int height, int sx, int sy, double cam_offs, double cam_side, double cam_up, double tan_fov, double ang, double pitch)
{
	// forward direction in the plane
	const double fwd_z = -sin(pitch);
	const double fwd_u = cos(pitch);
	const double fwd_x = sin(ang);
	const double fwd_y = cos(ang);

	// find position of camera
	const double cam_x = 0.5 * w - cam_offs * fwd_x * fwd_u + cam_side * fwd_y;
	const double cam_y = 0.5 * d - cam_offs * fwd_y * fwd_u - cam_side * fwd_x;
	const double cam_z = -cam_offs * fwd_z + cam_up;

	const double side_x = (double)sx - 0.5 * width;
	const double scrn_z = 0.5 * width / tan_fov;
	struct VoxEdge e = GetVoxelEdge(fwd_x, fwd_y, fwd_z, fwd_u, side_x, 0.5 * height, scrn_z, cam_x, cam_y, cam_z, w, d);

	if (e.x0>=0 && e.x1>=0) {
		const int dx = e.x1-e.x0;
		const int dy = e.y1-e.y0;
		if (abs(dx)>abs(dy)) {
			int s = dx > 0 ? 1 : -1;
			int scrn_p = height;
			for (int x = e.x0; (s*x)<(s*e.x1); x += s) {
				int y = e.y0 + (x-e.x0) * dy / dx;
				double h = hgt[x + y * w];
				double ox = (double)x-cam_x;
				double oy = (double)y-cam_y;
				double oz = h - cam_z;
				double cd = fwd_u * (fwd_x * ox + fwd_y * oy) + fwd_z * oz;
				if (cd > 0.01) {
					double ch = -fwd_z * (fwd_x * ox + fwd_y * oy) + fwd_u * oz;
					int scrn_y = clamp((int)(-ch / cd * scrn_z + 0.5 * height), 0, scrn_p);
					if (scrn_y<=sy) {
						hov_x = x;
						hov_y = y;
						return Pos2(x, y);
					}
				}
			}
		} else {
			int s = dy > 0 ? 1 : -1;
			int scrn_p = height;
			for (int y = e.y0; (s*y)<(s*e.y1); y += s) {
				int x = e.x0 + (y-e.y0) * dx / dy;
				double h = hgt[x + y * w];
				double ox = (double)x-cam_x;
				double oy = (double)y-cam_y;
				double oz = h - cam_z;
				double cd = fwd_u * (fwd_x * ox + fwd_y * oy) + fwd_z * oz;
				if (cd > 0.01) {
					double ch = -fwd_z * (fwd_x * ox + fwd_y * oy) + fwd_u * oz;
					int scrn_y = clamp((int)(-ch / cd * scrn_z + 0.5 * height), 0, scrn_p);
					if (scrn_y<=sy) {
						hov_x = x;
						hov_y = y;
						return Pos2(x, y);
					}
				}
			}
		}
	}

	return Pos2(-1, -1);
}

// find a screen position from a voxel map position
Pos2 Hill::ScreenProj(int width, int height, int x, int y, double cam_offs, double cam_side, double cam_up, double tan_fov, double ang, double pitch)
{
	// forward direction in the plane
	const double sp = sin(pitch);
	const double cp = cos(pitch);
	const double sa = sin(ang);
	const double ca = cos(ang);

	// find position of camera
	const double cam_x = 0.5 * w - cam_offs * sa * cp + cam_side * ca;
	const double cam_y = 0.5 * d - cam_offs * ca * cp - cam_side * sa;
	const double cam_z = cam_offs * sp + cam_up;

	const double ox = (double)x - cam_x;
	const double oy = (double)y - cam_y;
	const double oz = hgt[y * w + x] - cam_z;

	const double scrn_z = 0.5 * width / tan_fov;
	// the ground point in camera space
//	const double cx = fwd_y * ox - fwd_x * oy;
//	const double cy = -fwd_x * fwd_z * ox - fwd_y * fwd_z * oy + fwd_u * cam_z;
//	const double cz = fwd_x * fwd_u * ox + fwd_y * fwd_u * oy + fwd_z * cam_z;
	const double cx = ox * ca - (-cam_z) * sa;
	const double cy = ox * sa * cp - oy * sp + (-cam_z) * ca * cp;
	const double cz = -ox * sp * sa + oy * cp + (-cam_z) * sp * ca;

	const double m[3][3] = {
		{ ca, 0.0, -sa },
		{ sa * cp, -sp, ca * cp },
		{ -sa * sp, cp, ca * sp }
	};

	const double wx = cx * ca + cy * sa * cp - cz * sp * sa;
	const double wy = cy * (-sp) + cz * cp;
	const double wz = cx * (-sa) + cy * ca * cp + cz * sp * ca;


	const double dxy = m[0][0] * m[1][0] + m[0][1] * m[1][1] + m[0][2] * m[1][2];
	const double dyz = m[1][0] * m[2][0] + m[1][1] * m[2][1] + m[1][2] * m[2][2];
	const double dzx = m[2][0] * m[0][0] + m[2][1] * m[0][1] + m[2][2] * m[0][2];

	const double cxy[3] = {	m[0][1]*m[1][2] - m[0][2]*m[1][1], m[0][2]*m[1][0]-m[0][0]*m[1][2], m[0][0]*m[1][1] - m[0][1]*m[1][0] };
	const double cyz[3] = { m[1][1]*m[2][2] - m[1][2]*m[2][1], m[1][2]*m[2][0]-m[1][0]*m[2][2], m[1][0]*m[2][1] - m[1][1]*m[2][0] };
	const double czx[3] = { m[2][1]*m[0][2] - m[2][2]*m[0][1], m[2][2]*m[0][0]-m[2][0]*m[0][2], m[2][0]*m[0][1] - m[2][1]*m[0][0] };



	const double sx = cx * scrn_z / cz + 0.5 * width;
//	const double sy = cy * scrn_z / cz;

//	double h = hgt[x + y * w];
//	double ox = (double)x-cam_x;
//	double oy = (double)y-cam_y;
//	double oz = h - cam_z;
	double cd = cp * (sa * ox + ca * oy) - sp * oz;
	double ch = sp * (sa * ox + ca * oy) + cp * oz;
	double scrn_y = -ch / cd * scrn_z + 0.5 * height;

	//	const double dir_x = side * fwd_y + dist * fwd_x * fwd_u - hgt * fwd_z * fwd_x;
//	const double dir_y = -side * fwd_x + dist * fwd_y * fwd_u - hgt * fwd_z * fwd_y;
//	const double dir_z = dist * fwd_z - hgt * fwd_u;


	return Pos2((int)sx, (int)scrn_y);

	// x coord is from z = 0
	// y coord is from z = [vx,vy]
//	const double bx = fwd_x * fwd_u * x - fwd_y * fwd_u * y;
//	const double by = fwd_y * fwd_u * 

}
//#pragma optimize("", on)

//#pragma optimize("", off)
uint8_t* Hill::VoxelView(uint8_t *pImg, int width, int height, double cam_offs, double cam_side, double cam_up, double tan_fov, double ang, double pitch)
{
//	uint16_t *pDepth = depth ? (uint16_t*)calloc(1, width * height * sizeof(uint16_t)) : nullptr;

	// forward direction in the plane
	const double fwd_z = -sin(pitch);
	const double fwd_u = cos(pitch);
	const double fwd_x = sin(ang);
	const double fwd_y = cos(ang);

	// find position of camera
	const double cam_x = 0.5 * w - cam_offs * fwd_x * fwd_u + cam_side * fwd_y;
	const double cam_y = 0.5 * d - cam_offs * fwd_y * fwd_u - cam_side * fwd_x;
	const double cam_z = -cam_offs * fwd_z + cam_up;

	const double scrn_mid = 0.5 * width;
	const double scrn_dist = 0.5 * width / tan_fov;
	const double scrn_mid_hgt = 0.5 * height;

	const double sun_z = -0.25;
	const double sun_x = sqrt(1-sun_z*sun_z)/2;
	const double sun_y = sun_x;

	uint8_t amb_r = 48, amb_g = 48, amb_b = 16;

	// iterate left to right across screen
	for (int scrn_x = 0; scrn_x<width; scrn_x++) {
		// get a ray from the camera passing through the screen
		const double side_x = (double)scrn_x - scrn_mid;
		struct VoxEdge e = GetVoxelEdge(fwd_x, fwd_y, fwd_z, fwd_u, side_x, scrn_mid_hgt, scrn_dist, cam_x, cam_y, cam_z, w, d);
		if (e.x0>=0 && e.x1>=0) {
			const int lx0 = e.x0;
			const int ly0 = e.y0;
			const int lx1 = e.x1;
			const int ly1 = e.y1;
			const int dx = lx1-lx0;
			const int dy = ly1-ly0;

			if (abs(dx)>abs(dy)) {
				// iterate along x
				int s = dx > 0 ? 1 : -1;
				int scrn_p = height;
				for (int x = lx0; (s*x)<(s*lx1); x += s) {
					int y = ly0 + (x-lx0) * dy / dx;
					double h = hgt[x + y * w];
					double ox = (double)x-cam_x;
					double oy = (double)y-cam_y;
					double oz = h - cam_z;
					double cd = fwd_u * (fwd_x * ox + fwd_y * oy) + fwd_z * oz;
					if (cd > 0.01) {
						double ch = -fwd_z * (fwd_x * ox + fwd_y * oy) + fwd_u * oz;
						int scrn_y = clamp((int)(-ch / cd * scrn_dist + 0.5 * height), 0, scrn_p);
						if (scrn_y < scrn_p) {
							if (scrn_p==height)
								scrn_p = scrn_y+1;
							struct Normal n = GetNormalAtPoint(x, y);
							double l = -(n.x * sun_x + n.y * sun_y + n.z * sun_z);
							uint8_t r, g, b;
							if (abs(x-hov_x)<2 && abs(y-hov_y)<2) {
								int a = 0;
								a = a;
							}
							if (abs(x-hov_x)<3 || abs(y-hov_y)<3) {
								r = 255;
								g = 0;
								b = 255;
							} else if (l>=0.0) {
								r = (uint8_t)(l * (255-amb_r)) + amb_r;
								g = (uint8_t)(l * (255-amb_g)) + amb_g;
								b = (uint8_t)(l * (255-amb_b)) + amb_b;
							} else {
								r = amb_r;
								g = (uint8_t)(-l * (64-amb_g)) + amb_g;
								b = (uint8_t)(-l * (96-amb_b)) + amb_b;
							}
							uint8_t *pPix = pImg + (scrn_x + scrn_y * width) * 3;
							for (int draw_y = scrn_y; draw_y < scrn_p; ++draw_y) {
								pPix[0] = r; pPix[1] = g; pPix[2] = b;
								pPix += width * 3;
							}
							scrn_p = scrn_y;
						}
					}
				}
			} else {
				// iterate along y
				int s = dy > 0 ? 1 : -1;
				int scrn_p = height;
				for (int y = ly0; (s*y)<(s*ly1); y += s) {
					int x = lx0 + (y-ly0) * dx / dy;
					double h = hgt[x + y * w];
					double ox = (double)x-cam_x;
					double oy = (double)y-cam_y;
					double oz = h - cam_z;
					double cd = fwd_u * (fwd_x * ox + fwd_y * oy) + fwd_z * oz;
					if (cd > 0.01) {
						double ch = -fwd_z * (fwd_x * ox + fwd_y * oy) + fwd_u * oz;
						int scrn_y = clamp((int)(-ch / cd * scrn_dist + 0.5 * height), 0, scrn_p);
						if (scrn_y < scrn_p) {
							if (scrn_p==height)
								scrn_p = scrn_y+1;
							struct Normal n = GetNormalAtPoint(x, y);
							double l = -(n.x * sun_x + n.y * sun_y + n.z * sun_z);
							uint8_t r, g, b;
							if (abs(x-hov_x)<2 && abs(y-hov_y)<2) {
								int a = 0;
								a = a;
							}
							if (abs(x-hov_x)<3 || abs(y-hov_y)<3) {
								r = 255;
								g = 0;
								b = 255;
							} else if (l>=0.0) {
								r = (uint8_t)(l * (255-amb_r)) + amb_r;
								g = (uint8_t)(l * (255-amb_g)) + amb_g;
								b = (uint8_t)(l * (255-amb_b)) + amb_b;
							} else {
								r = amb_r;
								g = (uint8_t)(-l * (64-amb_g)) + amb_g;
								b = (uint8_t)(-l * (96-amb_b)) + amb_b;
							}
							uint8_t *pPix = pImg + (scrn_x + scrn_y * width) * 3;
							for (int draw_y = scrn_y; draw_y < scrn_p; ++draw_y) {
								pPix[0] = r; pPix[1] = g; pPix[2] = b;
								pPix += width * 3;
							}
							scrn_p = scrn_y;
						}
					}
				}
			}
		}
	}
/*	if (depth)
		*depth = pDepth;
*/	return pImg;
}
//#pragma optimize("", on)


#define ISO_SCALE 0.25
uint8_t* Hill::IsometricView(int &width, int &height)
{
	// 1x -> 1x, +0.5y
	// 1y -> 1x, -0.5y

	int pix_wide = w + d;
	double max_vert = 0.0;
	double min_vert = 0.0;
double max_h = -1e30;
	double *read = hgt;
	for (int y = 0; y<d; y++) {
		double base_depth_y = -ISO_SCALE * y;
		for (int x = 0; x<w; x++) {
			double base_width_y = ISO_SCALE * x;
			double h = *read++;
			double pos_y = base_depth_y + base_width_y - h;
			if (h > max_h)
				max_h = h;
			if (pos_y > max_vert)
				max_vert = pos_y;
			if (pos_y < min_vert)
				min_vert = pos_y;
		}
	}

	min_vert -= 128.0;
	int pix_high = (int)(max_vert-min_vert + 1.0) + 128;	// add some room
	uint8_t *buf = (uint8_t*)calloc(1, 3 * pix_wide * pix_high);
	if (!buf)
		return nullptr;

	width = pix_wide;
	height = pix_high;

	int max_v = pix_high-1;

	double SunX = (double)-w;
	double SunY = -0.5 * d;
	double SunZ = max_h * 2.0;

	for (int y = d-1; y>=0; --y) {
		int pos_depth_x = y;
		double base_depth_y = -ISO_SCALE * y;
		double prev_h = read[-1];
		for (int x = w-1; x>=0; --x) {
			int pos_width_x = x;
			double base_width_y = ISO_SCALE * x;
			double h = *--read;
			double prev_v = (y<(d-1)) ? read[w] : h;
			double next_v = y ? read[-w] : h;
			double next_h = read[-1];
			double pos_y = base_depth_y + base_width_y - h;
			int top = clamp((int)(pos_y-min_vert), 0, max_v);
			int bot = clamp((int)(base_depth_y + base_width_y-min_vert), 0, max_v);
			if (top>bot) {
				top = top ^ bot;
				bot = bot ^ top;
				top = top ^ bot;
			}

			// center height is C
			// neighbour heights are A, B, D, E
			//
			//       A
			//       | \
			//    D__C__B
			//     \ |
			//       E
			//
			// tri1: (0,0,C)-(0,1,A)-(1,0,B)
			// tri2: (0,0,C)-(0,-1,E)-(-1,0,D)
			// tri1 e1: (0-0, 1-0, A-C) = (0,1,A-C)
			// tri1 e2: (1-0, 0-1, B-A) = (1,-1,B-A)
			// tri2 e1: (0-0, -1-0, E-C) = (0,-1,E-C)
			// tri2 e2: (-1-9,0-1,D-E) = (-1,1,D-E)
			// tri1 e1xe2: (B-A)+(A-C), (A-C), -1 = B-C, A-C, -1
			// tri2 e1xe2: -(D-E)-(E-C), -(E-C), -1 = C-D, C-E, -1
			// Normal 1 = (C-E, C-D, 1)
			// Normal 2 = (A-C, B-C, 1)
			// average and normalize
			// invert winding order because up is +Z
			double Anx = next_h - prev_h;
			double Any = next_v - prev_v;
			double Anz = 2.0;
			double div = 1.0 / sqrt(Anx*Anx+Any*Any+Anz*Anz);
			Anx *= div;
			Any *= div;
			Anz *= div;

			double SoX = (double)x-SunX;
			double SoY = (double)y-SunY;
			double SoZ = (double)h-SunZ;
			double SoD = 1.0 / sqrt(SoX*SoX+SoY*SoY+SoZ*SoZ);
			SoX *= SoD;
			SoY *= SoD;
			SoZ *= SoD;

			double light = -(Anx * SoX + Any * SoY + Anz * SoZ);
			if (light < 0.0)
				light = 0.0;
			double sl = light * 0.75 + 0.25; // gray .25 is ambient
			uint8_t sl8 = (uint8_t)(255.0 * sl);

			uint8_t cr = sl8 * y / d;
			uint8_t cg = sl8 * x / w;
			uint8_t cb = uint8_t(255.0  * sl * (h>=0.0f ? h/max_h : 0.0f));
			uint8_t *pP = buf + 3 * (pos_width_x + pos_depth_x);
			int lm = pix_wide * 3;
//			int v = top;
			for (int v = top; v<bot; v++) {
				uint8_t *pD = pP + lm * v;
				*pD++ = cr;
				*pD++ = cg;
				*pD++ = cb;
			}
			prev_h = h;
		}
	}

	return buf;
}

Hill *Start_Hills()
{
	srand((unsigned int)time(NULL));

	Hill *pH = CreateMap(4096, 4096);


	//	pH->SimpleHill(0.75, 0.75, 0.1, 1024.0);
	pH->DomeHill(512, 0.25);
	return pH;
}


void Build_Hills(Hill *pH)
{
	srand((unsigned int)time(NULL));
	for (int i = 0; i<2; i++) {
		for (int d = 0; d<200; d++) {
			double weight = pow(i ? 0.95 : 0.97, d);
			pH->RandomDent(2.5, 300 * weight, pH->w * 0.1, pH->w * 0.5);
		}
		int cracks = i ? 150 : 500;

		for (int d = 0; d<i; d++) {
			double weight = pow(i ? 0.98 : 0.999, d);
			pH->RandomCrack(pH->w*0.5, pH->w*2.0, 50.0 * weight, 150.0 * weight);
		}

		//	uint8_t *pVox = pH->VoxelView(nullptr, 1024, 512, 2048.0, 0.0, 1.0, 3*M_PI_4, M_PI/8.0);
		//	stbi_write_png("vox.png", 1024, 512, 3, pVox, 0);
		//	free(pVox);
		//	exit(0);

		pH->GaussianBlur(i ? 50.0 : 150.0, 1.0);
	}

	for (int d = 0; d<40000 && !pH->stop; d++)
		pH->RandomCrack(pH->w*0.125, pH->w*0.5, 0.25, 1.25);

	pH->GaussianBlur(2.0, 1.0);

	//	pH->RadialAvg(24, 1.0);
	//	for (int d = 0; d<1000; d++)
	//		pH->RandomCrack(pH->w*0.125, pH->w*0.375, 0.25, 1.5);

	for (int d = 0; d<60000 && !pH->stop; d++)
		pH->RandomCrack(pH->w*0.015, pH->w*0.25, 0.05, 0.25);

#if 0	
	//	pH->Blur(32, 0.99995);
	//	pH->RadialAvg(24, 1.0);

	//	pH->RadialAvg(12, 1.0);
	pH->GaussianBlur(3.0, 1.0);

	for (int d = 0; d<10000 && !pH->stop; d++)
		pH->RandomCrack(pH->w*0.05, pH->w*0.25, 0.025, 0.1);

	//	for (int d=0; d<50000; d++)
	//		pH->RandomDent(0.15, 0.5);

	//	pH->RadialAvg(16, 0.75);

	Pos2List holes;
	pH->FindHoles(12, 1.0, holes);
#endif
	pH->WriteHgtMap("test.png");

	if (!pH->stop) {
		int iso_wid, iso_hgt;
		uint8_t *pIsoImg = pH->IsometricView(iso_wid, iso_hgt);

		stbi_write_png("iso.png", iso_wid, iso_hgt, 3, pIsoImg, 0);
	}
}


void Test_Hills()
{
	srand((unsigned int)time(NULL));

	Hill *pH = CreateMap(4096, 4096);

	
//	pH->SimpleHill(0.75, 0.75, 0.1, 1024.0);
	pH->DomeHill(512, 0.25);

//	for (int d = 0; d<1000; d++)
//		pH->RandomDent(1.0, 25.0);

	for (int d = 0; d<20; d++)
		pH->RandomDent(2.5, 250, pH->w * 0.1, pH->w * 0.5);

	for (int d = 0; d<25; d++)
		pH->RandomCrack(pH->w*0.5, pH->w*2.0, 100.0, 150.0);

//	uint8_t *pVox = pH->VoxelView(nullptr, 1024, 512, 2048.0, 0.0, 1.0, 3*M_PI_4, M_PI/8.0);
//	stbi_write_png("vox.png", 1024, 512, 3, pVox, 0);
//	free(pVox);
//	exit(0);

	pH->GaussianBlur(50.0, 1.0);


#if 0
	for (int d = 0; d<64; d++)
		pH->RandomDent(2.5, 100, pH->w * 0.1, pH->w * 0.5);

	for (int d = 0; d<20; d++)
		pH->RandomCrack(pH->w*0.5, pH->w*2.0, 25.0, 75.0);

	pH->GaussianBlur(5.0, 1.0);
#endif
	for (int d = 0; d<30000; d++)
		pH->RandomCrack(pH->w*0.125, pH->w*0.5, 0.5, 2.5);

	pH->GaussianBlur(3.0, 1.0);

//	pH->RadialAvg(24, 1.0);
//	for (int d = 0; d<1000; d++)
//		pH->RandomCrack(pH->w*0.125, pH->w*0.375, 0.25, 1.5);

	for (int d = 0; d<60000; d++)
		pH->RandomCrack(pH->w*0.125, pH->w*0.5, 0.05, 0.25);

#if 0	
	//	pH->Blur(32, 0.99995);
//	pH->RadialAvg(24, 1.0);

//	pH->RadialAvg(12, 1.0);
	pH->GaussianBlur(3.0, 1.0);

	for (int d = 0; d<10000; d++)
		pH->RandomCrack(pH->w*0.05, pH->w*0.25, 0.025, 0.1);

//	for (int d=0; d<50000; d++)
//		pH->RandomDent(0.15, 0.5);

//	pH->RadialAvg(16, 0.75);

	Pos2List holes;
	pH->FindHoles(12, 1.0, holes);
#endif
	pH->WriteHgtMap("test.png");

	int iso_wid, iso_hgt;
	uint8_t *pIsoImg = pH->IsometricView(iso_wid, iso_hgt);

	stbi_write_png("iso.png", iso_wid, iso_hgt, 3, pIsoImg, 0);

//	uint8_t *pVox = pH->VoxelView(nullptr, 1024, 512, 2048.0, 0.0, 1.0, 3*M_PI_4, M_PI/8.0);
//	stbi_write_png("vox.png", 1024, 512, 3, pVox, 0);
//	free(pVox);

	delete pH;
	exit(0);
}

#if 0
int main(int argc, const char * argv[]) {
	// insert code here...
	
	Hill *pH = CreateMap(2048, 2048);
	
	pH->SimpleHill(0.5, 0.5, 0.1, 512.0);
	
	for (int d=0; d<250000; d++)
		pH->RandomDent(0.25, 2.0);
	
	pH->WriteHgtMap("/Users/Carl-Henrik/Google Drive/code/Hillsen/hgtmap.png");
	
	delete pH;
	
    return 0;
}
#endif