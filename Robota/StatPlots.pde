import org.gicentre.utils.stat.*;        // For chart classes.
import controlP5.*;
BarChart barChart;
ArrayList<Float> Riv;

class Statplots extends PApplet {
  Statplots() {
    super();
    PApplet.runSketch(new String[] {this.getClass().getSimpleName()}, this);
  }

  void settings() {
    size(600, 400);
    smooth();
  }

  void setup() {  
    background(255);
    barChart = new BarChart(this);
    barChart.setData(new float[] {0, 0, 0, 0, 0, 0, 0});
    barChart.setBarLabels(new String[] {"   Passive", "   Reactive", "K=10, D=1", "K=14, D=2", "K=18, D=3", "K=26, D=4", "K=3, D=5"});
    barChart.setBarColour(color(0, 66, 66, 200));
    barChart.setBarGap(10); 
    barChart.setValueFormat("###.###### s");
    barChart.showValueAxis(true); 
    barChart.showCategoryAxis(true);
  }

  void draw() {
    background(255);
    barChart.draw(20, 90, width-20, height-100);
    fill(120);
    textSize(25);
    text("Time taken to cross the finish line", 120, 50);
    textSize(12);
    float textHeight = textAscent();
    text("Two systems compared; Passive (Given K,D value) and Reactive", 140, 50+textHeight);
  }

  void update(ArrayList<Float> R, ArrayList<Float> P, ArrayList<Float> KD1, ArrayList<Float> KD2, ArrayList<Float> KD3, ArrayList<Float> KD4, ArrayList<Float> KD5) {
    barChart.setData(new float[] {getAverage(P)/1000, getAverage(R)/1000, getAverage(KD1)/1000, getAverage(KD2)/1000, getAverage(KD3)/1000, getAverage(KD4)/1000, getAverage(KD5)/1000});
  }
}

float getAverage(ArrayList<Float> list) {
  // Do the sum
  float sum = 0;
  for (float n : list)
  {
    sum += n;
  }
  return sum/list.size();
}

float getVariance(ArrayList<Float> list)
{
  float mean = getAverage(list);
  float temp = 0;
  for (float a : list)
    temp += (a-mean)*(a-mean);
  return temp/(list.size()-1);
}

float getStdDev(ArrayList<Float> list)
{
  return sqrt(getVariance(list));
}