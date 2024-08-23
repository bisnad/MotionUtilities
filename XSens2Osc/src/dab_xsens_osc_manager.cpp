#include "dab_xsens_osc_manager.h"
#include "dab_xsens_mocap_skeleton.h"

using namespace dab;
using namespace dab::xsens;

# pragma mark OscSender definition

OscSender::OscSender(const std::string& pIpAddress, unsigned int pPort, float pSendRate)
	: mIpAddress(pIpAddress)
	, mPort(pPort)
{
	mSendInterval = unsigned int(1000000 / pSendRate);

	mSender.setup(mIpAddress, mPort);
}

OscSender::~OscSender()
{}

void 
OscSender::start()
{
	if (isThreadRunning() == false) startThread();
}

void 
OscSender::stop()
{
	if (isThreadRunning() == true) stopThread();
}

void 
OscSender::send()
{
	mLock.lock();

	try
	{
		MocapSkeletonManager& skeletonManager = MocapSkeletonManager::get();

		const std::vector<unsigned int>& skeletonIds = skeletonManager.skeletonIds();

		for (auto skel_id : skeletonIds)
		{
			std::shared_ptr<MocapSkeleton> skeleton = skeletonManager.skeleton(skel_id);
			const std::vector<std::string>& propertyNames = skeleton->propertyNames();

			for (auto properyName : propertyNames)
			{
				std::vector<float> property = skeleton->property(properyName);

				ofxOscMessage oscMessage;

				std::string address = std::string("/skel/") + std::to_string(skel_id) + properyName;

				oscMessage.setAddress(address);

				for (float value : property)
				{
					oscMessage.addFloatArg(value);
				}

				mSender.sendMessage(oscMessage, false);

			}
		}
	}
	catch (dab::Exception& e)
	{
		std::cout << e << "\n";
	}

	mLock.unlock();
}

void 
OscSender::threadedFunction()
{
	while (isThreadRunning())
	{
		send();

		std::this_thread::sleep_for(std::chrono::microseconds(mSendInterval));
	}
}


# pragma mark OscManager definition


OscManager::OscManager()
{}

OscManager::~OscManager() 
{
	stop();
}

void 
OscManager::addSender(const std::string& pIpAddress, unsigned int pPort, float pSendRate)
{
	std::shared_ptr<OscSender> sender( new OscSender(pIpAddress, pPort, pSendRate) );
	mSenders.push_back(sender);
}

void 
OscManager::start()
{
	for(auto sender : mSenders)
	{ 
		sender->start();
	}
}

void 
OscManager::stop()
{
	for (auto sender : mSenders)
	{
		sender->stop();
	}
}

