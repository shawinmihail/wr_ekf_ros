void CONTROL::LLtoNE(double dLat, double dLon, double*NE) {
  // Перевод географических координат (широты и долготы) точки в прямоугольные
  // координаты проекции Гаусса-Крюгера (на примере координат Москвы).

  // Географические координаты точки (в градусах)
  //dLon = 37.618  // Долгота (положительная для восточного полушария)
  //dLat = 55.752  // Широта (положительная для северного полушария)

  // Номер зоны Гаусса-Крюгера (если точка рассматривается в системе
  // координат соседней зоны, то номер зоны следует присвоить вручную)
  int zone = (int)dLon/6+1;
  //printf("Zone: %i\n", zone);

  // Параметры эллипсоида Красовского
  double a = 6378245.0;          // Большая (экваториальная) полуось
  double b = 6356863.019;       // Малая (полярная) полуось
  double e2 = (a*a-b*b)/(a*a);  // Эксцентриситет
  double n = (a-b)/(a+b);        // Приплюснутость

  // Параметры зоны Гаусса-Крюгера
  double F = 1.0;           		// Масштабный коэффициент
  double Lat0 = 0.0;                // Начальная параллель (в радианах)
  double Lon0 = (zone*6-3)*PI/180;  // Центральный меридиан (в радианах)
  double N0 = 0.0;                  // Условное северное смещение для начальной параллели
  double E0 = zone*1000000+500000.0;    // Условное восточное смещение для центрального меридиана

  // Перевод широты и долготы в радианы
  double Lat = dLat*PI/180.0;
  double Lon = dLon*PI/180.0;

  // Вычисление переменных для преобразования
  double sinLat = sin(Lat);
  double cosLat = cos(Lat);
  double tanLat = tan(Lat);

  double v = a*F*pow(1-e2*pow(sinLat,2),-0.5);
  double p = a*F*(1-e2)*pow(1-e2*pow(sinLat,2),-1.5);
  double n2 = v/p-1;
  double M1 = (1+n+5.0/4.0*pow(n,2)+5.0/4.0*pow(n,3))*(Lat-Lat0);
  double M2 = (3*n+3*pow(n,2)+21.0/8.0*pow(n,3))*sin(Lat - Lat0)*cos(Lat + Lat0);
  double M3 = (15.0/8.0*pow(n,2)+15.0/8.0*pow(n,3))*sin(2*(Lat - Lat0))*cos(2*(Lat + Lat0));
  double M4 = 35.0/24.0*pow(n,3)*sin(3*(Lat - Lat0))*cos(3*(Lat + Lat0));
  double M = b*F*(M1-M2+M3-M4);
  double I = M+N0;
  double II = v/2 * sinLat * cosLat;
  double III = v/24 * sinLat * pow(cosLat,3) * (5-pow(tanLat,2)+9*n2);
  double IIIA = v/720 * sinLat * pow(cosLat,5) * (61-58*pow(tanLat,2)+pow(tanLat,4));
  double IV = v * cosLat;
  double V = v/6 * pow(cosLat,3) * (v/p-pow(tanLat,2));
  double VI = v/120 * pow(cosLat,5) * (5-18*pow(tanLat,2)+pow(tanLat,4)+14*n2-58*pow(tanLat,2)*n2);

  // Вычисление северного и восточного смещения (в метрах)
  double N = I+II*pow(Lon-Lon0,2)+III*pow(Lon-Lon0,4)+IIIA*pow(Lon-Lon0,6);
  double E = E0+IV*(Lon-Lon0)+V*pow(Lon-Lon0,3)+VI*pow(Lon-Lon0,5);

  NE[0] = N;
  NE[1] = E;
}