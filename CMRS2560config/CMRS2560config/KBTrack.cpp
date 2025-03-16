#include "KBTrack.h"

KBTrack::KBTrack()
{
	int i;
	for (i = 0; i < 16; i++)
	{
		mode[i] = 0;
	}
}

KBTrack::~KBTrack()
{

}

void KBTrack::set_mode(int index, int arg)
{
	mode[index] = arg;
}

int KBTrack::get_mode(int index)
{
	return mode[index];
}

void KBTrack::clear() {
	int i;
	for (i = 0; i < 16; i++)
	{
		mode[i] = 0;
	}
}