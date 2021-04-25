//
// Created by coumarc9 on 7/1/17.
//

#ifndef PROVIDER_HYDROPHONE_PING_H
#define PROVIDER_HYDROPHONE_PING_H

namespace provider_hydrophone {
    class Ping {

    public:
        Ping();
        ~Ping();

        void setFrequency(unsigned char frequency) {this->frequency = frequency;};
        unsigned char getFrequency(){return this->frequency;};

        void setAmplitude(unsigned int amplitude) {this->amplitude = amplitude;};
        unsigned int getAmplitude(){return this->amplitude;};

        void setNoise(unsigned int noise) {this->noise = noise;};
        unsigned int getNoise(){return this->noise;};

        void setChannelReferenceReal(int channelReferenceReal) {this->channelReferenceReal = channelReferenceReal;};
        int getChannelReferenceReal(){return this->channelReferenceReal;};

        void setChannelReferenceImage(int channelReferenceImage) {this->channelReferenceImage = channelReferenceImage;};
        int getChannelReferenceImage(){return this->channelReferenceImage;};

        void setChannel1Real(int channel1Real) {this->channel1Real = channel1Real;};
        int getChannel1Real(){return this->channel1Real;};

        void setChannel1Image(int channel1Image) {this->channel1Image = channel1Image;};
        int getChannel1Image(){return this->channel1Image;};

        void setChannel2Real(int channel2Real) {this->channel2Real = channel2Real;};
        int getChannel2Real(){return this->channel2Real;};

        void setChannel2Image(int channel2Image) {this->channel2Image = channel2Image;};
        int getChannel2Image(){return this->channel2Image;};

    private:

        unsigned char frequency; // frequency in kHz
        unsigned int amplitude;
        unsigned int noise;

        int channelReferenceReal;
        int channelReferenceImage;

        int channel1Real;
        int channel1Image;

        int channel2Real;
        int channel2Image;

    };
}


#endif //PROVIDER_HYDROPHONE_PING_H
