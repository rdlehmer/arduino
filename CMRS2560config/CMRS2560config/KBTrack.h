#pragma once
class KBTrack
{
public:

	KBTrack();
	~KBTrack();

	void set_mode(int, int);
	int get_mode(int);

private:
	int mode[16];
};

