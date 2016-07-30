// Retinex.cpp
// Retinex PDE algorithm implementation for OpenCV
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   16.09.2013
// Description: This class implements the Retinex PDE algorithm for OpenCV. Implementation is
//              based on Nicolas Limare et al. published implementation, with some modifications
//              to be integrated in a C++ class and to interface the class with OpenCV.
// References: Nicolas Limare, Ana Belén Petro, Catalina Sbert, Jean-Michel Morel (2011)
//             Retinex Poisson Equation: a Model for Color Perception
//             In IPOL Journal - Image Processing On Line
//             http://www.ipol.im/pub/art/2011/lmps_rpe/
//             OpenCV 2.4.6 - http://docs.opencv.org/index.html
///////////////////////////////////////////////////////////////////////////////////////////////////

#include "Retinex.h"

// == Construction and destruction ================================================================

CRetinex::CRetinex(void) {
	IsImageLoaded = false;
	IsResultAvailable = false;
	Threshold = 0.0;
	IsPlanCreated = false;
}


CRetinex::~CRetinex(void) {
	// TODO: Free allocated memory if required
	// Destroy plans if defined
	RemoveFFTWPlans();
}

// == Algorithm control methods ===================================================================

void CRetinex::LoadImage(Mat &im) {
	// Load the image to be processed and prepare algorithm data
	it = im.type();
	nc = im.channels();
	nx = im.cols;
	ny = im.rows;
	data.resize(nc);
	data_rtnx.resize(nc);
	SrcFChannels.resize(nc);
	ResFChannels.resize(nc);
	SrcImage = im.clone();
	ResImage = SrcImage.clone();
	//split(SrcImage,SrcChannels);
	//imshow("pr2",SrcChannels[1]);
	SplitImage();
	for (int i=0; i<nc; i++) {
		//ResChannels[i].push_back(SrcChannels[i]);
		ResFChannels[i] = SrcFChannels[i];
		data[i] = (float *) &(SrcFChannels[i][0]);
		data_rtnx[i] = (float *) &(ResFChannels[i][0]);
	}
	// Reset status flags
	IsImageLoaded = true;
	IsResultAvailable = false;
	// Create plans for this image type
	if (!IsPlanCreated)
		CreateFFTWPlans();
}

void CRetinex::SplitImage() {
	int cy = SrcImage.step[0];
	int cx = SrcImage.step[1];
	for (int i=0; i<nc; i++)
		SrcFChannels[i].resize(nx*ny);
	for (int i = 0; i < ny; i++) {
		for(int j = 0; j < nx; j += nc) {
			for (int n=0; n<nc; n++)
				SrcFChannels[n][i*nx+j] = (float) SrcImage.data[i*cy + j*cx + n];
		}
	}
}

void CRetinex::MergeImage() {
	int cy = ResImage.step[0];
	int cx = ResImage.step[1];
	for (int i = 0; i < ny; i++) {
		for(int j = 0; j < nx; j += nc) {
			for (int n=0; n<nc; n++)
				ResImage.data[i*cy + j*cx + n] = (unsigned int) ResFChannels[n][i*nx+j];
		}
	}
}

void CRetinex::SetThreshold(float t) {
	// Set the Retinex threshold (it must be in the (0..255) interval)
	if (t>0.0 && t<255.0)
		Threshold = t;
}

float CRetinex::GetThreshold() {
	// Get the currently active Retinex threshold
	return Threshold;
}

void CRetinex::CreateFFTWPlans() {
	// Create plans for FFTW before actually calculating DFT
	// An image must be loaded
	if (IsImageLoaded) {
		// Prepare memory buffers
		vdata_fft.clear();
		vdata_fft.resize(nx*ny);
		data_fft = &(vdata_fft[0]);
		vdata_tmp.clear();
		vdata_tmp.resize(nx*ny);
		data_tmp = &(vdata_tmp[0]);
		// TODO: Check pointers
		// Prepare resulting data pointer for plans
		data_res = data_rtnx[0];
		// Create forward plan
	    dct_fw = fftwf_plan_r2r_2d((int) ny, (int) nx,
                               data_tmp, data_fft,
                               FFTW_REDFT10, FFTW_REDFT10,
                               FFTW_MEASURE | FFTW_DESTROY_INPUT);
		// Create backward plan
		dct_bw = fftwf_plan_r2r_2d((int) ny, (int) nx,
                               data_fft, data_res,
                               FFTW_REDFT01, FFTW_REDFT01,
                               FFTW_MEASURE | FFTW_DESTROY_INPUT);
		// Indicate that plans exist
		IsPlanCreated = true;
	}
}

void CRetinex::RemoveFFTWPlans() {
	if (IsPlanCreated) {
		fftwf_destroy_plan(dct_fw);
		fftwf_destroy_plan(dct_bw);
		IsPlanCreated = false;
	}
	fftwf_cleanup();
}

void CRetinex::Run() {
	float *res = NULL;
	// Execute algorithm
	IsResultAvailable = false;
	for (int i=0; i<nc; i++) {
		data_res = data_rtnx[i];
		res = RetinexPDE(data_res,nx,ny,Threshold);
		if (res==NULL)
			break;
		NormalizeMeanDt(data_res,data[i],nx*ny);
	}
	//merge(ResChannels,ResImage);
	//ResImage.convertTo(ResImage,it);
	MergeImage();
	IsResultAvailable = true;
}

// -- Retinex PDE implementation ------------------------------------------------------------------
// This function solves the Retinex PDE equation with forward and
// backward DCT. The input array is processed as follows:
//   1. A discrete laplacian is computed with a threshold
//   2. This discrete laplacian array is symmetrized in both directions
//   3. This data is transformed by forward DFT (both steps can be
//      handled by a simple DCT)
//   4. u(i,j) is calculated
//   5. This data is transformed by backward DFT
//
// Parameters:
//   data       input/output array
//   nx, ny     dimensions
//   t          the retinex threshold
// Result:
//   data, or NULL if an error occured

float *CRetinex::RetinexPDE(float *data, size_t nx, size_t ny, float t) {
    //fftwf_plan dct_fw, dct_bw;
    //float *data_fft, *data_tmp;

    // Checks and initialisation
    // Check allocation
    if (NULL == data) {
        fprintf(stderr, "a pointer is NULL and should not be so\n");
		// TODO: Remove this abort()
        abort();
    }
    // Start threaded FFTW if FFTW_NTHREADS is defined
#ifdef FFTW_NTHREADS
    if (0 == fftwf_init_threads()) {
        fprintf(stderr, "fftw initialisation error\n");
        abort();
    }
    fftwf_plan_with_nthreads(FFTW_NTHREADS);
#endif
    // Allocate the float-complex FFT array and the float tmp array
  //  if (NULL == (data_fft = (float *) malloc(sizeof(float) * nx * ny))
  //      || NULL == (data_tmp = (float *) malloc(sizeof(float) * nx * ny))) {
  //      fprintf(stderr, "allocation error\n");
		//// TODO: Remove this abort()
  //      abort();
  //  }

	// Retinex PDE
	// Compute the laplacian: data -> data_tmp
    (void) DiscreteLaplacianThreshold(data_tmp, data, nx, ny, t);
    // Create the DFT forward plan and run the DCT : data_tmp -> data_fft
    //dct_fw = fftwf_plan_r2r_2d((int) ny, (int) nx,
    //                           data_tmp, data_fft,
    //                           FFTW_REDFT10, FFTW_REDFT10,
    //                           FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
    fftwf_execute(dct_fw);
    //free(data_tmp);

    // Solve the Poisson PDE in Fourier space
    // 1.0 / (float) (nx * ny)) is the DCT normalisation term, see libfftw
    (void) RetinexPoissonDCT(data_fft, nx, ny, 1.0 / (double) (nx * ny));
    // Create the DFT backward plan and run the iDCT : data_fft -> data
    //dct_bw = fftwf_plan_r2r_2d((int) ny, (int) nx,
    //                           data_fft, data,
    //                           FFTW_REDFT01, FFTW_REDFT01,
    //                           FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
    fftwf_execute(dct_bw);
    // Cleanup
    //fftwf_destroy_plan(dct_fw);
    //fftwf_destroy_plan(dct_bw);
    //free(data_fft);
    //fftwf_cleanup();
#ifdef FFTW_NTHREADS
    fftwf_cleanup_threads();
#endif
    return data;
}

// == Helper methods ==============================================================================

void CRetinex::MeanDt(const float *data, size_t size, double *mean_p, double *dt_p) {
    double mean, dt;
    const float *ptr_data;
    size_t i;

    mean = 0.0;
    dt = 0.0;
    ptr_data = data;
    for (i = 0; i < size; i++) {
        mean += *ptr_data;
        dt += (*ptr_data) * (*ptr_data);
        ptr_data++;
    }
    mean /= (double) size;
    dt /= (double) size;
    dt -= (mean * mean);
    dt = sqrt(dt);
    *mean_p = mean;
    *dt_p = dt;
}

void CRetinex::NormalizeMeanDt(float *data, const float *ref, size_t size) {
    double mean_ref, mean_data, dt_ref, dt_data;  
    double a, b;
    size_t i;
    float *ptr_data;

    // Sanity check
    if (NULL == data || NULL == ref) {
        fprintf(stderr, "a pointer is NULL and should not be so\n");
		// TODO: Remove this abort()
        abort();
    }
    // Compute mean and variance of the two arrays
    MeanDt(ref, size, &mean_ref, &dt_ref);
    MeanDt(data, size, &mean_data, &dt_data);
    // Compute the normalization coefficients
    a = dt_ref / dt_data;
    b = mean_ref - a * mean_data;
    // Normalize the array
    ptr_data = data;
    for (i = 0; i < size; i++) {
        *ptr_data = a * *ptr_data + b;
        ptr_data++;
    }
}

// -- Discrete Laplacian with Threshold -----------------------------------------------------------
// Compute the discrete laplacian of a 2D array with a threshold:
//     (F_{i - 1, j} - F_{i, j})
//     + (F_{i + 1, j} - F_{i, j})
//     + (F_{i, j - 1} - F_{i, j})
//     + (F_{i, j + 1} - F_{i, j})
// On the border, differences with "outside of the array" are 0.
// If the absolute value of difference is < t, 0 is used instead.
//
// This step takes a significant part of the computation time, and
// needs to be fast. In that case, we observed that (with our compiler
// and architecture):
// - pointer arithmetic is faster than data[i]
// - if() is faster than ( ? : )
//
// Parameters:
//   data_out  output array
//   data_in   input array
//   nx, ny    array size
//   t         threshold
// Result:
//   data_out  Pointer to resulting laplacian

float *CRetinex::DiscreteLaplacianThreshold(float *data_out, const float *data_in, size_t nx, size_t ny, float t) {
    size_t i, j;
    float *ptr_out;
    float diff;
    // Pointers to the current and neighbour values
    const float *ptr_in, *ptr_in_xm1, *ptr_in_xp1, *ptr_in_ym1, *ptr_in_yp1;

    // Sanity check
    if (NULL == data_in || NULL == data_out) {
        fprintf(stderr, "a pointer is NULL and should not be so\n");
		// TODO: Remove this abort()
        abort();
    }

    // Pointers to the data and neighbour values
    //
    //                 y-1
    //             x-1 ptr x+1
    //                 y+1
    //    <---------------------nx------->

    ptr_in = data_in;
    ptr_in_xm1 = data_in - 1;
    ptr_in_xp1 = data_in + 1;
    ptr_in_ym1 = data_in - nx;
    ptr_in_yp1 = data_in + nx;
    ptr_out = data_out;
    // Iterate on j, i, following the array order
    for (j = 0; j < ny; j++) {
        for (i = 0; i < nx; i++) {
            *ptr_out = 0.;
            // Row differences
            if (0 < i) {
                diff = *ptr_in - *ptr_in_xm1;
                if (fabs(diff) > t)
                    *ptr_out += diff;
            }
            if (nx - 1 > i) {
                diff = *ptr_in - *ptr_in_xp1;
                if (fabs(diff) > t)
                    *ptr_out += diff;
            }
            // Column differences
            if (0 < j) {
                diff = *ptr_in - *ptr_in_ym1;
                if (fabs(diff) > t)
                    *ptr_out += diff;
            }
            if ( ny - 1 > j) {
                diff = *ptr_in - *ptr_in_yp1;
                if (fabs(diff) > t)
                    *ptr_out += diff;
            }
            ptr_in++;
            ptr_in_xm1++;
            ptr_in_xp1++;
            ptr_in_ym1++;
            ptr_in_yp1++;
            ptr_out++;
        }
    }
    return data_out;
}

// -- Compute a cosinus table ---------------------------------------------------------------------
// Allocate and fill a table of n values cos(i Pi / n) for i in [0..n]
// Parameters:
//   size   The size of table to be generated
// Result:
//   Pointer to allocated and filled table

double *CRetinex::CosTable(size_t size) {
    double *table = NULL;
    double *ptr_table;
    size_t i;

    // Allocate the cosinus table
    if (NULL == (table = (double *) malloc(sizeof(double) * size))) {
        fprintf(stderr, "allocation error\n");
        abort();
    }
    // Fill the cosinus table:
    // table[i] = cos(i Pi / n) for i in [0..n]
    ptr_table = table;
    for (i = 0; i < size; i++)
        *ptr_table++ = cos((M_PI * i) / size);
    return table;
}

// -- Poisson PDE ---------------------------------------------------------------------------------
// Perform a Poisson PDE in the Fourier DCT space:
//   u(i, j) = F(i, j) * m / (4 - 2 cos(i PI / nx) - 2 cos(j PI / ny))  for (i, j) != (0, 0)
//   u(0, 0) = 0
//
// When this function is successively used on arrays of identical
// size, the trigonometric computation is redundant and could be kept
// in memory for a faster code. However, in our use case, the speedup
// is marginal and we prefer to recompute this data and keep the code
// simple.
//
// Parameters:
//   data       the dct complex coefficients, of size nx x ny
//   nx, ny     data array size
//   m          global multiplication parameter (DCT normalization)
// Result:
//   Updated data array

float *CRetinex::RetinexPoissonDCT(float *data, size_t nx, size_t ny, double m) {
    float *ptr_data;
    double *cosi = NULL, *cosj = NULL;
    double *ptr_cosi, *ptr_cosj;
    size_t i, j;
    double m2;

    // Get the cosinus tables
    // cosi[i] = cos(i Pi / nx) for i in [0..nx]
    // cosj[j] = cos(j Pi / ny) for j in [0..ny]
    cosi = CosTable(nx);
    cosj = CosTable(ny);

    // We will now multiply data[i, j] by m / (4 - 2 * cosi[i] - 2 * cosj[j]))
    // and set data[i, j] to 0
    m2 = m / 2.0;
    ptr_data = data;
    ptr_cosi = cosi;
    ptr_cosj = cosj;

    // Handle the first value, data[0, 0] = 0
    // After that, by construction, we always have *ptr_cosi + *ptr_cosj != 2.
    *ptr_data++ = 0.0;
    ptr_cosi++;
    // Continue with the first line from the second value
    for (i = 1; i < nx; i++)
		*ptr_data++ *= m2 / (2. - *ptr_cosi++ - *ptr_cosj);
    ptr_cosj++;
    ptr_cosi = cosi;
    // Continue with the other lines
    for (j = 1; j < ny; j++) {
        for (i = 0; i < nx; i++)
            *ptr_data++ *= m2 / (2. - *ptr_cosi++ - *ptr_cosj);
        ptr_cosj++;
        ptr_cosi = cosi;
    }
    free(cosi);
    free(cosj);
    return data;
}
