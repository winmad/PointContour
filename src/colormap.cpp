/*************************************************************************
*
* ADOBE PROPRIETARY INFORMATION
*
* Use is governed by the license in the attached LICENSE.TXT file
*
*
*  Copyright 2010 Adobe Systems Incorporated
*  All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains
* the property of Adobe Systems Incorporated and its suppliers,
* if any.  The intellectual and technical concepts contained
* herein are proprietary to Adobe Systems Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents,
* patents in process, and are protected by trade secret or copyright law.
* Dissemination of this information or reproduction of this material
* is strictly forbidden unless prior written permission is obtained
* from Adobe Systems Incorporated.
**************************************************************************/


#include "colormap.h"

void Colormap::colormapBSC(unsigned int n, std::vector<Colormap::color> &colors)
{
	unsigned binSize = (unsigned)(ceil((float)(n/6.0f)));
	Colormap::color c;
	c.r = 0;c.g = 0;c.b = 1;
	colors.push_back(c);
	c.r = 0;c.g = 1;c.b = 0;
	colors.push_back(c);
	c.r = 1;c.g = 0;c.b = 0;
	colors.push_back(c);
	c.r = 0;c.g = 1;c.b = 1;
	colors.push_back(c);
	c.r = 1;c.g = 0;c.b = 1;
	colors.push_back(c);
	c.r = 1;c.g = 1;c.b = 0;
	colors.push_back(c);

	for (unsigned int i=1; i< binSize; ++i){
		for (unsigned int j=0; j< 6; ++j){
			c.r = colors[j].r * double(i)/double(binSize) ;
			c.g = colors[j].g * double(i)/double(binSize) ;
			c.b = colors[j].b * double(i)/double(binSize) ;
			colors.push_back(c);
		}
	}
}

void Colormap::colormapHSV(unsigned int n, std::vector<Colormap::color> &colors)
{

		unsigned binSize = (unsigned)(ceil((float)(n/6.0f)));

		// from red to yellow
		// increase green intensity
		for (unsigned int i=0; i< binSize; ++i){
			Colormap::color c;
			c.r = 1.0f;			
			c.g = ((float)i/(float)binSize);
			c.b = 0.0f;
			colors.push_back(c);
		}
		
		// from yellow to green
		// decrease red intensity
		for (unsigned int i=binSize; i >0 ; --i){
			Colormap::color c;
			c.r = ((float)i/(float)binSize);
			c.g = 1.0f;
			c.b = 0.0f;
			colors.push_back(c);
		}

		// from green to cyan
		// increase blue intensity
		for (unsigned int i=0; i< binSize; ++i){
			Colormap::color c;
			c.r = 1.0f;
			c.g = 0.0f;
			c.b =((float)i/(float)binSize);
			//color c(0.0f,1.0f, (i/binSize));
			colors.push_back(c);
		}

		// from cyan to blue
		// decrease green intensity
		for (unsigned int i=binSize; i >0 ; --i){
			Colormap::color c;
			c.r = 0.0f;
			c.g = ((float)i/(float)binSize);
			c.b = 1.0f;
			//color c(0.0f, (i/binSize), 1.0f);
			colors.push_back(c);
		}

		// from blue to magenta
		// increase red intensity
		for (unsigned int i=0; i< binSize; ++i){
			Colormap::color c;
			c.r = ((float)i/(float)binSize);
			c.g = 0.0f;
			c.b = 1.0f;
			//color c((i/binSize), 0.0f, 1.0f);
			colors.push_back(c);
		}

		// from magenta to red
		// decrease blue intensity
		for (unsigned int i=binSize; i > 0 ; --i){
			Colormap::color c;
			c.r = 1.0f;
			c.g = 0.0f;
			c.b = ((float)i/(float)binSize);
			//color c(1.0f, 0.0, (i/binSize));
			colors.push_back(c);
		}
}
void Colormap::colormapJet1(unsigned int n, std::vector<color> &colors)
{
		unsigned binSize = (unsigned)(ceil((float)(n/4.0f)));

		// from Blue to Cyan		
		for (unsigned int i=0; i< binSize; ++i){
			Colormap::color c;
			c.r = 0.0f;			
			c.g = ((float)i/(float)binSize);
			c.b = 1.0f;
			colors.push_back(c);
		}
		
		// from Cyan to Green
		for (unsigned int i=binSize; i >0 ; --i){
			Colormap::color c;
			c.r = 0.0f;
			c.g = 1.0f;
			c.b = ((float)i/(float)binSize);
			colors.push_back(c);
		}

		// from Green to Yellow
		for (unsigned int i=0; i< binSize; ++i){
			Colormap::color c;
			c.r = ((float)i/(float)binSize);
			c.g = 1.0f;
			c.b = 0.0f;
			colors.push_back(c);
		}

		// from Yellow to Red
		// decrease green intensity
		for (unsigned int i=binSize; i >0 ; --i){
			Colormap::color c;
			c.r = 1.0f;
			c.g = ((float)i/(float)binSize);
			c.b = 0.0f;
			colors.push_back(c);
		}


}
void Colormap::colormapJet2(unsigned int n, std::vector<color> &colors)
{
		unsigned binSize = (unsigned)(ceil((float)(n/4.0f)));

		// from R to Y		
		for (unsigned int i=0; i< binSize; ++i){
			Colormap::color c;
			c.r = 1.0f;			
			c.g = ((float)i/(float)binSize);
			c.b = 0.0f;
			colors.push_back(c);
		}
		
		// from Y to Green
		for (unsigned int i=binSize; i > 0 ; --i){
			Colormap::color c;
			c.r = ((float)i/(float)binSize);
			c.g = 1.0f;
			c.b = 0.0f;
			colors.push_back(c);
		}

		// from Green to C
		for (unsigned int i=0; i< binSize; ++i){
			Colormap::color c;
			c.r = 0.0f;
			c.g = 1.0f;
			c.b = ((float)i/(float)binSize);
			colors.push_back(c);
		}

		// from C to B
		for (unsigned int i=binSize; i >0 ; --i){
			Colormap::color c;
			c.r = 0.0f;
			c.g = ((float)i/(float)binSize);
			c.b = 1.0f;
			colors.push_back(c);
		}


}
void Colormap::colormapWinter(unsigned int n, std::vector<Colormap::color> &colors){

	// from Green to Blue : 2 bins
	
	unsigned binSize = (unsigned)(ceil((float)(n/2.0f)));

		// from Green to Cyan
		// increase green intensity
		for (unsigned int i=0; i< binSize; ++i){
			Colormap::color c;
			c.r = 0.0f;
			c.g = 1.0f;
			c.b = ((float)i/(float)binSize);
			colors.push_back(c);
		}
		
		// from Cyan to Blue
		// decrease green intensity
		for (unsigned int i = binSize; i > 0 ; --i){
			Colormap::color c;
			c.r = 0.0;
			c.g = ((float)i/(float)binSize);
			c.b = 1.0f;
			colors.push_back(c);
		}


}
void Colormap::colormapHeatColor(unsigned int n, std::vector<color> &colors)
{
	unsigned binSize = (unsigned)(ceil((float)(n/4.0f)));

	// from R to Y		
	for (unsigned int i=0; i< binSize; ++i){
		Colormap::color c;
		c.r = 1.0f;			
		c.g = ((float)i/(float)binSize);
		c.b = 0.0f;
		colors.push_back(c);
	}

	// from Y to Green
	for (unsigned int i=binSize; i > 0 ; --i){
		Colormap::color c;
		c.r = ((float)i/(float)binSize);
		c.g = 1.0f;
		c.b = 0.0f;
		colors.push_back(c);
	}

	// from Green to C
	for (unsigned int i=0; i< binSize; ++i){
		Colormap::color c;
		c.r = 0.0f;
		c.g = 1.0f;
		c.b = ((float)i/(float)binSize);
		colors.push_back(c);
	}

	// from C to B
	for (unsigned int i=binSize; i >0 ; --i){
		Colormap::color c;
		c.r = 0.0f;
		c.g = ((float)i/(float)binSize);
		c.b = 1.0f;
		colors.push_back(c);
	}
}
void Colormap::colormapHeatColorReverse(unsigned int n, std::vector<color> &colors)
{
	unsigned binSize = (unsigned)(ceil((float)(n/4.0f)));

	// from Blue to Cyan		
	for (unsigned int i=0; i< binSize; ++i){
		Colormap::color c;
		c.r = 0.0f;			
		c.g = ((float)i/(float)binSize);
		c.b = 1.0f;
		colors.push_back(c);
	}

	// from Cyan to Green
	for (unsigned int i=binSize; i >0 ; --i){
		Colormap::color c;
		c.r = 0.0f;
		c.g = 1.0f;
		c.b = ((float)i/(float)binSize);
		colors.push_back(c);
	}

	// from Green to Yellow
	for (unsigned int i=0; i< binSize; ++i){
		Colormap::color c;
		c.r = ((float)i/(float)binSize);
		c.g = 1.0f;
		c.b = 0.0f;
		colors.push_back(c);
	}

	// from Yellow to Red
	// decrease green intensity
	for (unsigned int i=binSize; i >0 ; --i){
		Colormap::color c;
		c.r = 1.0f;
		c.g = ((float)i/(float)binSize);
		c.b = 0.0f;
		colors.push_back(c);
	}
}


void Colormap::HslToRgb(std::vector<double>& mag, std::vector<color> &colors)
{
	HSL hsl; hsl.Y=1.0;hsl.Z=0.5;
	color tcolor;
	float& r=tcolor.r;
	float& g=tcolor.g;
	float& b=tcolor.b;

	for(unsigned i=0;i<mag.size();i++){
		hsl.X=mag[i];

		if (hsl.Y == 0.0f)
			r = g = b = hsl.Z;
		else
		{
			double q = hsl.Z < 0.5f ? hsl.Z * (1.0f + hsl.Y) : hsl.Z + hsl.Y - hsl.Z * hsl.Y;
			double p = 2.0f * hsl.Z - q;
			r = HueToRgb(p, q, hsl.X + 1.0f / 3.0f);
			g = HueToRgb(p, q, hsl.X);
			b = HueToRgb(p, q, hsl.X - 1.0f / 3.0f);
		}

		colors.push_back(tcolor);
	}

}

// Helper for HslToRgba
double Colormap::HueToRgb(double p, double q, double t)
{
	if (t < 0.0) t += 1.0f;
	if (t > 1.0) t -= 1.0f;
	if (t < 1.0 / 6.0) return p + (q - p) * 6.0 * t;
	if (t < 1.0 / 2.0) return q;
	if (t < 2.0 / 3.0) return p + (q - p) * (2.0/3.0 - t) * 6.0;
	return p;
}