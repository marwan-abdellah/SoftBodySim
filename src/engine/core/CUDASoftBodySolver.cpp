#include "CUDASoftBodySolver.h"

using namespace std;

#define VAR(v,w) __typeof(w) v=w
#define FOREACH(it,c) for(VAR(it,(c).begin());it!=(c).end();++it)

#define DBG(fmt, ...) fprintf(stderr, "\033[22;32m:%s:%d:" fmt, __func__, __LINE__, ##__VA_ARGS__)
#define ERR(fmt, ...) fprintf(stderr, "\033[01;31m:%s:%d:" fmt, __func__, __LINE__, ##__VA_ARGS__)
#define WRN(fmt, ...) fprintf(stderr, "\033[01;33m:%s:%d:" fmt, __func__, __LINE__, ##__VA_ARGS__)


CUDASoftBodySolver::CUDASoftBodySolver(void)
	:
		m_initialized(false)
{
}

CUDASoftBodySolver::~CUDASoftBodySolver(void)
{
	if (m_initialized)
		terminate();
}

bool CUDASoftBodySolver::initializeDevice(void)
{
	cudaError_t error;
	int device_count, dev_id;

	error = cudaGetDeviceCount(&device_count);
	if (error != cudaSuccess)
		return false;

	// take by default last device
	dev_id = device_count - 1;

	if (dev_id < 0 || dev_id >= device_count)
		return false;

	error = cudaSetDevice(dev_id);
	if (error != cudaSuccess)
		return false;
	
	error = cudaStreamCreate(&m_stream);
	if (error != cudaSuccess)
		return false;

	return true;
}

bool CUDASoftBodySolver::serializeBodies(vector<SoftBody> &bodies)
{
	int bytes_to_alloc = 0;
	int idx = 0, idx2= 0;
	cudaError_t error;

	FOREACH(it, bodies)
		bytes_to_alloc += it->m_vertexes.size() * sizeof(glm::vec3);

	for (int type = ARRAY_POSITIONS; type < ARRAY_LAST_DEFINED; ++type) {
		error = cudaMalloc(&m_array[type], bytes_to_alloc);
		if (error != cudaSuccess)
			goto on_error;
	}

	error = cudaMalloc(&m_mass_inv, bytes_to_alloc);
	if (error != cudaSuccess)
		goto on_error;

	cudaMemset(m_array[ARRAY_FORCES], 0x0, bytes_to_alloc);

	bytes_to_alloc = 0;
	FOREACH(it, bodies)
		bytes_to_alloc += it->m_links.size();

	error = cudaMalloc(&m_links, bytes_to_alloc * sizeof(glm::uvec2));
	if (error != cudaSuccess) {
		goto on_error;
	}
	cudaMalloc(&m_links_rest_length2, bytes_to_alloc * sizeof(glm::float_t));
	if (error != cudaSuccess) {
		goto on_error;
	}

	FOREACH(it, bodies) {
		SoftBodyDescriptor descr;
		descr.body = &(*it);
		descr.vertex_base_idx = idx;
		descr.links_base_idx = idx2;

		idx += it->m_vertexes.size();
		idx2 += it->m_links.size();

		unsigned int bytes1 = it->m_vertexes.size() * sizeof(glm::vec3);
		unsigned int bytes2 = it->m_links.size() * sizeof(glm::uvec2);
		unsigned int bytes3 = it->m_links.size() * sizeof(glm::float_t);

		unsigned int offset = idx * sizeof(glm::vec3);
		unsigned int offset2 = idx2 * sizeof(glm::uvec2);
		unsigned int offset3 = idx2 * sizeof(glm::float_t);

		cudaMemcpy(m_array[ARRAY_POSITIONS] + offset, &it->m_vertexes[0], bytes1, cudaMemcpyHostToDevice);
		cudaMemcpy(m_array[ARRAY_PROJECTIONS] + offset, &(it->m_vertexes[0]), bytes1, cudaMemcpyHostToDevice);
		cudaMemcpy(m_array[ARRAY_VELOCITIES] + offset, &(it->m_velocities[0]), bytes1, cudaMemcpyHostToDevice);
		cudaMemcpy(m_array[ARRAY_FORCES] + offset, &(it->m_forces[0]), bytes1, cudaMemcpyHostToDevice);
		cudaMemset(m_mass_inv + offset3, it->m_mass_inv, bytes1);
		//cudaMemcpy(m_links + offset2, &(it->m_links.indexes[0]), bytes2, cudaMemcpyHostToDevice);
		//cudaMemcpy(m_links_rest_length2 + offset3, &(it->m_links.restLength[0]), bytes3, cudaMemcpyHostToDevice);

		m_descriptors.push_back(descr);
	}

	return true;

on_error:
	ERR("Unable to allocate memory for soft bodies. %s", cudaGetErrorString(error));
	for (int type = ARRAY_POSITIONS; type < ARRAY_LAST_DEFINED; type++) {
		if (m_array[type]) cudaFree(m_array[type]);
		m_array[type] = NULL;
	}

	if (m_links) cudaFree(m_links);
	m_links = NULL;
	if (m_links_rest_length2) cudaFree(m_links_rest_length2);
	m_links_rest_length2 = NULL;
	m_descriptors.clear();
	return false;
}

void CUDASoftBodySolver::initialize(vector<SoftBody> &bodies)
{
	if (!initializeDevice())
		ERR("CUDA Device initialization failed!");

	if (!serializeBodies(bodies))
		ERR("Unable to allocate resources on device!");

	m_initialized = true;
}

void CUDASoftBodySolver::shutdownDevice(void)
{
	cudaStreamDestroy(m_stream);
}

void CUDASoftBodySolver::terminate(void)
{
	shutdownDevice();
}

