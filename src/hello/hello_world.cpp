#include <iostream>
#include <vector>

struct sampleData {
	int id_;
    int value_;
};

class DataSampler
{
private:
	std::vector<sampleData> data_;
	
public:
	DataSampler()
	{
		data_.clear();
	}

	void addDataSample(sampleData &sample)
	{
		data_.push_back(sample);
	}

	bool getSample(int id, sampleData &sample)
	{
		for(auto& s : data_)
		{
			if(id == s.id_)
			{
				sample = s;
				return true;
			}
		}

		return false;
	}

	bool mutateN(int id, int value)
	{
		sampleData sample;
		if (getSample(id, sample))
		{
			sample.value_ = value;
		}
	}

	int getValue(int id)
	{
		for(auto& s : data_)
		{
			if(id == s.id_)
				return s.value_;
		}
		return -999;
	}
};

int main(int argc, char *argv[]) {
	DataSampler sampler = DataSampler();
    sampleData a {1, 2};
    sampleData b {10, 20};
    sampler.addDataSample(a);
    sampler.addDataSample(b);
    sampler.mutateN(1, 5);
    std:: cout << sampler.getValue(1) << std::endl;
}