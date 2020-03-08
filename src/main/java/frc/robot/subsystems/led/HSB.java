package frc.robot.subsystems.led;

public class HSB {
   
    public static double[] rgb2hsv(double r, double g, double b)
    {
        double h, s ,v;
        double min, max, delta;
        min = Math.min(Math.min(r, g), b);
        max = Math.max(Math.max(r, g), b);
        v = max;                                // v
        delta = max - min;
        if (delta < 0.00001)
        {
            s = 0;
            h = 0; // undefined, maybe nan?
            return new double[]{h,s,v};
        }
        if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
            s = (delta / max);                  // s
        } else {
            // if max is 0, then r = g = b = 0              
            // s = 0, h is undefined
            s = 0.0;
            h = Double.NaN;                            // its now undefined
            return new double[]{h,s,v};
        }
        if( r >= max )                           // > is bogus, just keeps compilor happy
            h = ( g - b ) / delta;        // between yellow & magenta
        else
        if( g >= max )
            h = 2.0 + ( b - r ) / delta;  // between cyan & yellow
        else
            h = 4.0 + ( r - g ) / delta;  // between magenta & cyan

        h *= 60.0;                              // degrees

        if( h < 0.0 )
            h += 360.0;

        return new double[]{h,s,v};
    }


    public static double[] hsv2rgb(double h, double s, double v)
    {
        double      hh, p, q, t, ff;
        long        i;
        double r, g, b;

        if(s <= 0.0) {       // < is bogus, just shuts up warnings
            r = v;
            g = v;
            b = v;
            return new double[]{r, g, b};
        }
        hh = h;
        if(hh >= 360.0) hh = 0.0;
        hh /= 60.0;
        i = (long)hh;
        ff = hh - i;
        p = v * (1.0 - s);
        q = v * (1.0 - (s * ff));
        t = v * (1.0 - (s * (1.0 - ff)));

        switch((int) i) {
            case 0:
                r = v;
                g = t;
                b = p;
                break;
            case 1:
                r = q;
                g = v;
                b = p;
                break;
            case 2:
                r = p;
                g = v;
                b = t;
                break;

            case 3:
                r = p;
                g = q;
                b = v;
                break;
            case 4:
                r = t;
                g = p;
                b = v;
                break;
            case 5:
            default:
                r = v;
                g = p;
                b = q;
                break;
        }
        return new double[]{r, g, b};
    }
}