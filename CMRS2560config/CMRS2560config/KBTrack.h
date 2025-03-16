#pragma once
class KBTrack
{
public:

	KBTrack();
	~KBTrack();

	void set_mode(int, int);
	int get_mode(int);

	void clear();

private:
	int mode[16];
};

