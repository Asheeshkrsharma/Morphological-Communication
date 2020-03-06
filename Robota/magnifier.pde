class magnifier {
  PGraphics mask;
  int x, y, w, h;
  magnifier(int x_, int y_, int w_, int h_) {
    x=x_;
    y=y_;
    w=w_;
    h=h_;
    mask=createGraphics(w, h);//draw the mask object
    mask.beginDraw();
    mask.stroke(5);
    mask.smooth();//this really does nothing, i wish it did
    mask.ellipse(w/2, h/2, w, h);
    mask.endDraw();
  }

  void display(float lensX, float lensY, Vec2 tmp) {
    int r=(int)sqrt(pow(lensX-tmp.x,2)+pow(lensY-tmp.y,2))+150;
    PImage img =  get((int)lensX-r/2, (int)lensY-r/2, r, r);

    img.resize(w, h);
    img.mask(mask);

    pushMatrix();
    imageMode(CENTER);
    translate(0, 0);
    image(img, x, y);
    popMatrix();
    noFill();
    strokeWeight(4);
    ellipse(x, y, w, h);
    fill(0);
  }

  void drag(int xx, int yy) {
    if (lensDrag) {
      boolean dragx=(xx<x+(w/2))||(xx>x-(w/2));
      boolean dragy=(yy<y+(w/2))||(yy>y-(w/2));
      if (xx>=x || dragx && yy>=y || dragy) {
        x=xx;
        y=yy;
      }
    }
  }
  
}