
Point barycentre(Polygon p){
    Point out;
    double x = 0, y = 0;
    for (auto item : p)
    {
    x += item.x;
    y += item.y;
    }
    out.x /= p.size();
    out.y /= p.size();
    return out;
}