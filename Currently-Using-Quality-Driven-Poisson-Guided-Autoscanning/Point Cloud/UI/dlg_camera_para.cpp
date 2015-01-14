#include "UI/dlg_camera_para.h"


CameraParaDlg::CameraParaDlg(QWidget *p, ParameterMgr * _paras, GLArea * _area) : QFrame(p)
{
  ui = new Ui::camera_paras;
  CameraParaDlg::ui->setupUi(this);
  area = _area;
  m_paras = _paras;

  if(!initWidgets())
  {
    cerr << " PoissonParaDlg::initWidgets failed." << endl;
    return;
  }
  initConnects();
}

void CameraParaDlg::initConnects()
{
  if (!connect(area,SIGNAL(needUpdateStatus()),this,SLOT(initWidgets())))
  {
    cout << "can not connect signal" << endl;
  }
  connect(ui->doubleSpinBox_view_prune_confidence_threshold, SIGNAL(valueChanged(double)), this, SLOT(getViewPruneConfidenceThreshold(double)));
  connect(ui->pushButton_view_prune, SIGNAL(clicked()), this, SLOT(runViewPrune()));
  connect(ui->pushButton_show_candidate_index, SIGNAL(clicked()), this, SLOT(showCandidateIndex()));
  connect(ui->pushButton_load_to_mesh, SIGNAL(clicked()), this, SLOT(loadToOriginal()));
  connect(ui->pushButton_load_to_model, SIGNAL(clicked()), this, SLOT(loadToModel()));
  connect(ui->checkBox_show_init_cameras,SIGNAL(clicked(bool)),this,SLOT(showInitCameras(bool)));
  connect(ui->checkBox_show_camera_border, SIGNAL(clicked(bool)), this, SLOT(showCameraBorder(bool)));
  connect(ui->pushButton_scan, SIGNAL(clicked()), this, SLOT(NBVCandidatesScanByHand()));
  connect(ui->pushButton_initial_scan, SIGNAL(clicked()), this, SLOT(initialScan()));
  connect(ui->horizon_dist, SIGNAL(valueChanged(double)), this, SLOT(getCameraHorizonDist(double)));
  connect(ui->vertical_dist, SIGNAL(valueChanged(double)), this, SLOT(getCameraVerticalDist(double)));
  connect(ui->dist_to_model, SIGNAL(valueChanged(double)), this, SLOT(getCameraDistToModel(double)));
  connect(ui->tableView_scan_candidates, SIGNAL(clicked(QModelIndex)), this, SLOT(showSelectedScannCandidates(QModelIndex)));
  connect(ui->tableView_scan_results, SIGNAL(clicked(QModelIndex)), this, SLOT(showSelectedScannedMesh(QModelIndex)));
  connect(ui->pushButton_merge_with_original, SIGNAL(clicked()), this, SLOT(mergeScannedMeshWithOriginalByHand()));
  connect(ui->doubleSpinBox_merge_confidence_threshold, SIGNAL(valueChanged(double)), this, SLOT(getMergeConfidenceThreshold(double)));
  connect(ui->doubleSpinBox_far_distance, SIGNAL(valueChanged(double)), this, SLOT(getCameraFarDistance(double)));
  connect(ui->doubleSpinBox_near_distance, SIGNAL(valueChanged(double)), this, SLOT(getCameraNearDistance(double)));
  connect(ui->doubleSpinBox_predicted_model_size, SIGNAL(valueChanged(double)), this, SLOT(getPredictedModelSize(double)));
  connect(ui->doubleSpinBox_optimal_plane_width, SIGNAL(valueChanged(double)), this, SLOT(getOptimalPlaneWidth(double)));
  connect(ui->ray_resolution_para, SIGNAL(valueChanged(double)), this, SLOT(getRayResolutionPara(double)));

  connect(ui->pushButton_build_grid, SIGNAL(clicked()), this, SLOT(buildGrid()));
  connect(ui->pushButton_propagate, SIGNAL(clicked()), this, SLOT(propagate()));
  connect(ui->pushButton_propagate_one, SIGNAL(clicked()), this, SLOT(propagateOnePoint()));
  connect(ui->pushButton_grid_segment, SIGNAL(clicked()), this, SLOT(gridSegment()));

  connect(ui->extract_view_candidates, SIGNAL(clicked()), this, SLOT(extractViewCandidates()));
  connect(ui->pushButton_extract_views_into_bins, SIGNAL(clicked()), this, SLOT(extractViewIntoBins()));
  connect(ui->cluster_view_candidates, SIGNAL(clicked()), this, SLOT(runViewClustering()));

  connect(ui->max_ray_steps, SIGNAL(valueChanged(double)), this, SLOT(getMaxRaySteps(double)));
  connect(ui->view_grid_resolution, SIGNAL(valueChanged(double)), this, SLOT(getGridResolution(double)));
  connect(ui->propagate_one_point_index, SIGNAL(valueChanged(double)), this, SLOT(getPropagateIndex(double)));

  connect(ui->use_other_inside_segment,SIGNAL(clicked(bool)),this,SLOT(useOtherInsideSegment(bool)));
  connect(ui->use_confidence_Separation,SIGNAL(clicked(bool)),this,SLOT(useConfidenceSeparation(bool)));
  connect(ui->need_more_overlaps,SIGNAL(clicked(bool)),this,SLOT(needMoreOverlaps(bool)));
  connect(ui->use_max_propagation, SIGNAL(clicked(bool)), this, SLOT(useMaxConfidencePropagation(bool)));
  connect(ui->doubleSpinBox_bottom_delta, SIGNAL(valueChanged(double)), this, SLOT(getIsoBottomDelta(double)));
  connect(ui->pushButton_set_iso_bottom_confidence, SIGNAL(clicked()), this, SLOT(runSetIsoBottomConfidence()));
  connect(ui->update_view_directions, SIGNAL(clicked()), this, SLOT(runUpdateViewDirections()));

  connect(ui->pushButton_setup_initial_scans, SIGNAL(clicked()), this, SLOT(runSetupInitialScanns()));
  connect(ui->step2_run_Poisson_Confidence_original, SIGNAL(clicked()), this, SLOT(runStep2PoissonConfidenceViaOriginal()));
  connect(ui->step2_run_Poisson_Confidence_original_2, SIGNAL(clicked()), this, SLOT(runStep2PoissonConfidenceViaOriginal()));
  connect(ui->step3_run_NBV, SIGNAL(clicked()), this, SLOT(runStep3NBVcandidates()));
  connect(ui->step3_run_NBV_2, SIGNAL(clicked()), this, SLOT(runStep3NBVcandidates()));
  connect(ui->step4_run_New_Scan, SIGNAL(clicked()), this, SLOT(runStep4NewScans()));
  connect(ui->pushButton_get_model_size, SIGNAL(clicked()), this, SLOT(getModelSize()));

  //sdf related
  connect(ui->pushButton_load_sdf_voxels, SIGNAL(clicked()), this, SLOT(loadSDFVoxels()));
  connect(ui->pushButton_run_sdf_slice, SIGNAL(clicked()), this, SLOT(runSDFSlice()));
  connect(ui->checkBox_show_sdf_slice_X,SIGNAL(clicked(bool)),this,SLOT(showSDFSliceX(bool)));
  connect(ui->checkBox_show_sdf_slice_Y,SIGNAL(clicked(bool)),this,SLOT(showSDFSliceY(bool)));
  connect(ui->checkBox_show_sdf_slice_Z,SIGNAL(clicked(bool)),this,SLOT(showSDFSliceZ(bool)));

  //auto scene related
  connect(ui->pushButton_load_scene, SIGNAL(clicked()), this, SLOT(loadScene()));
  connect(ui->pushButton_detect_plane, SIGNAL(clicked()), this, SLOT(detectPlane()));
  connect(ui->checkBox_pick_original, SIGNAL(clicked(bool)), this, SLOT(usePickOriginal(bool)));
  connect(ui->pushButton_compute_scene_confidence, SIGNAL(clicked()), this, SLOT(computeSceneConfidence()));
  connect(ui->pushButton_compute_scene_nbv, SIGNAL(clicked()), this, SLOT(computeSceneNBV()));
  connect(ui->pushButton_save_pickpoint_to_iso, SIGNAL(clicked()), this, SLOT(savePickPointToIso()));
  connect(ui->pushButton_detect_changed_points, SIGNAL(clicked()), this, SLOT(detectChangedPoints()));
   connect(ui->pushButton_update_graphcut, SIGNAL(clicked()), this, SLOT(updateGraphCut()));
  connect(ui->pushButton_test_graphcut, SIGNAL(clicked()), this, SLOT(runGraphCut()));
  connect(ui->pushButton_over_segment, SIGNAL(clicked()), this, SLOT(runOverSegmentation()));
  connect(ui->pushButton_segment_scene, SIGNAL(clicked()), this, SLOT(runSceneSegmentation()));
  
}

bool CameraParaDlg::initWidgets()
{
  ui->checkBox_show_camera_border->setChecked(m_paras->camera.getBool("Show Camera Border"));
  ui->doubleSpinBox_view_prune_confidence_threshold->setValue(m_paras->nbv.getDouble("View Prune Confidence Threshold"));
  ui->doubleSpinBox_merge_confidence_threshold->setValue(m_paras->camera.getDouble("Merge Confidence Threshold"));
  ui->horizon_dist->setValue(m_paras->camera.getDouble("Camera Horizon Dist"));
  ui->vertical_dist->setValue(m_paras->camera.getDouble("Camera Vertical Dist"));
  ui->dist_to_model->setValue(m_paras->camera.getDouble("Camera Dist To Model"));
  ui->view_grid_resolution->setValue(m_paras->nbv.getDouble("View Grid Resolution"));
  ui->max_ray_steps->setValue(m_paras->nbv.getDouble("Max Ray Steps Para"));
  ui->doubleSpinBox_far_distance->setValue(m_paras->camera.getDouble("Camera Far Distance"));
  ui->doubleSpinBox_near_distance->setValue(m_paras->camera.getDouble("Camera Near Distance"));
  ui->doubleSpinBox_predicted_model_size->setValue(m_paras->camera.getDouble("Predicted Model Size"));
  ui->doubleSpinBox_optimal_plane_width->setValue(m_paras->camera.getDouble("Optimal Plane Width"));
  ui->propagate_one_point_index->setValue(m_paras->nbv.getDouble("Propagate One Point Index"));
  ui->ray_resolution_para->setValue(m_paras->nbv.getDouble("Ray Resolution Para"));

  Qt::CheckState state = m_paras->nbv.getBool("Test Other Inside Segment") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->use_other_inside_segment->setCheckState(state);

  state = m_paras->nbv.getBool("Use Confidence Separation") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->use_confidence_Separation->setCheckState(state);

  state = m_paras->nbv.getBool("Need Update Direction With More Overlaps") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->need_more_overlaps->setCheckState(state);

  state = m_paras->nbv.getBool("Use Max Propagation") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->use_max_propagation->setCheckState(state);

  //sdf related
  state = m_paras->nbv.getBool("Show SDF Slice X") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->checkBox_show_sdf_slice_X->setCheckState(state);
  state = m_paras->nbv.getBool("Show SDF Slice Y") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->checkBox_show_sdf_slice_Y->setCheckState(state);
  state = m_paras->nbv.getBool("Show SDF Slice Z") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->checkBox_show_sdf_slice_Z->setCheckState(state);

  updateTableViewNBVCandidate();
  updateTabelViewScanResults();
  update();
  repaint();
  return true;
}

CameraParaDlg::~CameraParaDlg()
{
  delete ui;
  ui = NULL;
  area = NULL;
  m_paras = NULL;
}

void CameraParaDlg::setFrameConent()
{
  if(layout()) delete layout();
  QGridLayout * vLayout = new QGridLayout(this);
  vLayout->setAlignment(Qt::AlignTop);
  setLayout(vLayout);

  showNormal();
  adjustSize();
}

void CameraParaDlg::updateTableViewNBVCandidate()
{
  //init nbv scan candidates table view
  //add table header
  QStandardItemModel *model = new QStandardItemModel(); 
  model->setColumnCount(2); 
  model->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("position")); 
  model->setHeaderData(1,Qt::Horizontal,QString::fromLocal8Bit("direction"));
  //set table property
  ui->tableView_scan_candidates->setModel(model);
  //select the whole row
  ui->tableView_scan_candidates->setSelectionBehavior(QAbstractItemView::SelectRows);
  ui->tableView_scan_candidates->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
  ui->tableView_scan_candidates->horizontalHeader()->setResizeMode(0, QHeaderView::Stretch);
  ui->tableView_scan_candidates->horizontalHeader()->setResizeMode(1, QHeaderView::Stretch);
  ui->tableView_scan_candidates->setColumnWidth(0, 100);
  ui->tableView_scan_candidates->setColumnWidth(1, 100);

  //set table contents
  vector<ScanCandidate> *nbv_candidates = area->dataMgr.getScanCandidates();
  int i = 0;
  for (vector<ScanCandidate>::iterator it = nbv_candidates->begin(); 
    it != nbv_candidates->end(); ++it, ++i)
  {
    QString pos, direction;
    pos.sprintf("(%4.3f, %4.3f, %4.3f)", it->first.X(), it->first.Y(), it->first.Z());
    direction.sprintf("(%4.3f, %4.3f, %4.3f)", it->second.X(), it->second.Y(), it->second.Z());
    model->setItem(i, 0, new QStandardItem(pos));
    //model->item(i, 0)->setForeground(QBrush(QColor(255, 0, 0)));
    model->item(i, 0)->setTextAlignment(Qt::AlignCenter);
    model->setItem(i, 1, new QStandardItem(direction));
    model->item(i, 1)->setTextAlignment(Qt::AlignCenter);
  }
  ui->tableView_scan_results->clearSpans();
}

void CameraParaDlg::updateTabelViewScanResults()
{
  //init tableView_scan_results
  //add table header
  QStandardItemModel *model = new QStandardItemModel(); 
  model->setColumnCount(1); 
  model->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("scanned mesh")); 
  //set table property
  ui->tableView_scan_results->setContextMenuPolicy(Qt::CustomContextMenu);
  ui->tableView_scan_results->setModel(model);
  ui->tableView_scan_results->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
  ui->tableView_scan_results->horizontalHeader()->setResizeMode(0, QHeaderView::Stretch);
  ui->tableView_scan_results->setColumnWidth(0, 100);
  //set table contents
  vector<CMesh* > *rs = area->dataMgr.getScannedResults();
  vector<CMesh* >::iterator it = (*rs).begin();
  int i = 0;
  for (; it != (*rs).end(); ++it, ++i)
  {
    QString scanned_mesh_str;
    scanned_mesh_str.sprintf("Scanned mesh %d", i+1);
    model->setItem(i, 0, new QStandardItem(scanned_mesh_str));
    model->item(i, 0)->setTextAlignment(Qt::AlignLeft);
  }
}

void CameraParaDlg::initialScan()
{
  global_paraMgr.camera.setValue("Run Initial Scan", BoolValue(true));
  area->runCamera();
  global_paraMgr.camera.setValue("Run Initial Scan", BoolValue(false));

  updateTabelViewScanResults();
}

void CameraParaDlg::NBVCandidatesScan()
{
  //scan only the candidates those are chosen
  /*vector<ScanCandidate> *sc = area->dataMgr.getScanCandidates();
  vector<ScanCandidate> select_scan_candidates;
  QModelIndexList sil = ui->tableView_scan_candidates->selectionModel()->selectedRows();
  for (int i = 0; i < sil.size(); ++i)
  {
  int row = sil[i].row();
  select_scan_candidates.push_back((*sc)[row]);
  }
  sc->clear();
  std::copy(select_scan_candidates.begin(), select_scan_candidates.end(), std::back_inserter(*sc));*/

  global_paraMgr.camera.setValue("Run NBV Scan", BoolValue(true));
  area->runCamera();
  global_paraMgr.camera.setValue("Run NBV Scan", BoolValue(false));

  area->updateGL();
  updateTableViewNBVCandidate();
  updateTabelViewScanResults();
}

void CameraParaDlg::NBVCandidatesScanByHand()
{
  //scan only the candidates those are chosen
  vector<ScanCandidate> *sc = area->dataMgr.getScanCandidates();
  vector<ScanCandidate> select_scan_candidates;
  QModelIndexList sil = ui->tableView_scan_candidates->selectionModel()->selectedRows();
  for (int i = 0; i < sil.size(); ++i)
  {
    int row = sil[i].row();
    select_scan_candidates.push_back((*sc)[row]);
  }
  sc->clear();
  std::copy(select_scan_candidates.begin(), select_scan_candidates.end(), std::back_inserter(*sc));

  global_paraMgr.camera.setValue("Run NBV Scan", BoolValue(true));
  area->runCamera();
  global_paraMgr.camera.setValue("Run NBV Scan", BoolValue(false));

  area->updateGL();
  updateTableViewNBVCandidate();
  updateTabelViewScanResults();
}

void CameraParaDlg::showCandidateIndex()
{
  global_paraMgr.nbv.setValue("Run Compute View Candidate Index", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Compute View Candidate Index", BoolValue(false));
}

void CameraParaDlg::loadRealInitialScan()
{
  QString file_location = QFileDialog::getExistingDirectory(this, "choose a directory...", "",QFileDialog::ShowDirsOnly);
  if (!file_location.size()) 
    return;

  QDir dir(file_location);
  if (!dir.exists()) 
    return;

  dir.setFilter(QDir::Files);
  dir.setSorting(QDir::Name);
  QFileInfoList list = dir.entryInfoList();

  CMesh *original = area->dataMgr.getCurrentOriginal();

  for (int i = 0; i < list.size(); ++i)
  {
    QFileInfo fileInfo = list.at(i);
    QString f_name = fileInfo.fileName();

    if (!f_name.endsWith(".ply"))
      continue;

    f_name = file_location + "\\" + f_name;
    CMesh initial_scan;
    int mask = tri::io::Mask::IOM_VERTCOORD + tri::io::Mask::IOM_VERTNORMAL;
    if (i == 0)
    {
      area->dataMgr.loadPlyToOriginal(f_name);
    }else
    {
      CMesh* iso_points = area->dataMgr.getCurrentIsoPoints();
      tri::io::ImporterPLY<CMesh>::Open(*iso_points, f_name.toAscii().data(), mask);
      //GlobalFun::computeICP(original, iso_points);
      //tri::io::ImporterPLY<CMesh>::Open(initial_scan, f_name.toAscii().data(), mask);
      //GlobalFun::computeICP(original, &initial_scan);
    }
  }
}

void CameraParaDlg::loadToOriginal()
{
  QString file = QFileDialog::getOpenFileName(this, "Select a ply file", "", "*.ply");
  if(!file.size()) return;

  area->dataMgr.loadPlyToOriginal(file);

  area->dataMgr.normalizeAllMesh();
  area->initView();
  area->initAfterOpenFile();
  area->updateGL();
  std::cout<<"max normalize length: " <<global_paraMgr.data.getDouble("Max Normalize Length") <<endl;
}

void CameraParaDlg::loadToModel()
{
  QString file = QFileDialog::getOpenFileName(this, "Select a ply file", "", "*.ply");
  if(!file.size()) return;

  area->dataMgr.loadPlyToModel(file);
  vcg::tri::UpdateNormals<CMesh>::PerFace(*area->dataMgr.getCurrentModel());
  int f_size = area->dataMgr.getCurrentModel()->face.size();
  for (int f = 0; f < f_size; ++f)
    area->dataMgr.getCurrentModel()->face[f].N().Normalize();

  area->dataMgr.normalizeAllMesh();
  area->initView();
  area->initAfterOpenFile();
  area->updateGL();
}

void CameraParaDlg::virtualScan()
{
  global_paraMgr.camera.setValue("Run Virtual Scan", BoolValue(true));
  area->runCamera();
  global_paraMgr.camera.setValue("Run Virtual Scan", BoolValue(false));
}

void CameraParaDlg::showInitCameras(bool is_show)
{
  if (is_show)
  {
    global_paraMgr.camera.setValue("Is Init Camera Show", BoolValue(true));
    vector<ScanCandidate> *seletedViewCameras  = area->dataMgr.getSelectedScanCandidates();
    vector<ScanCandidate> *initViewCameras = area->dataMgr.getInitCameraScanCandidates();
    seletedViewCameras->clear();

    std::copy(initViewCameras->begin(), initViewCameras->end(), std::back_inserter((*seletedViewCameras)));
  }else
  {
    global_paraMgr.camera.setValue("Is Init Camera Show", BoolValue(false));
    vector<ScanCandidate> *seletedViewCameras  = area->dataMgr.getSelectedScanCandidates();
    seletedViewCameras->clear();
  }
}

void CameraParaDlg::showCameraBorder(bool is_show)
{
  if (is_show)
  {
    global_paraMgr.camera.setValue("Show Camera Border", BoolValue(true));
  }else
  {
    global_paraMgr.camera.setValue("Show Camera Border", BoolValue(false));
  }
}

void CameraParaDlg::showSDFSliceX(bool _val)
{
  global_paraMgr.nbv.setValue("Show SDF Slice X", BoolValue(_val));
  area->updateGL();
}

void CameraParaDlg::showSDFSliceY(bool _val)
{
  global_paraMgr.nbv.setValue("Show SDF Slice Y", BoolValue(_val));
  area->updateGL();
}

void CameraParaDlg::showSDFSliceZ(bool _val)
{
  global_paraMgr.nbv.setValue("Show SDF Slice Z", BoolValue(_val));
  area->updateGL();
}

void CameraParaDlg::usePickOriginal(bool _val)
{
  global_paraMgr.drawer.setValue("Use Pick Original", BoolValue(_val));
  area->updateGL();
}

void CameraParaDlg::useOtherInsideSegment(bool _val)
{
  global_paraMgr.nbv.setValue("Test Other Inside Segment", BoolValue(_val));
}

void CameraParaDlg::useConfidenceSeparation(bool _val)
{
  global_paraMgr.nbv.setValue("Use Confidence Separation", BoolValue(_val));
  cout << "Use Confidence Separation" << endl;
}

void CameraParaDlg::needMoreOverlaps(bool _val)
{
  global_paraMgr.nbv.setValue("Need Update Direction With More Overlaps", BoolValue(_val));
  cout << "Need Update Direction With More Overlaps" << endl;
}

void CameraParaDlg::useMaxConfidencePropagation(bool _val)
{
  global_paraMgr.nbv.setValue("Use Max Propagation", BoolValue(_val));
  cout << "Use Max Propagation" << endl;
}

void CameraParaDlg::showSelectedScannCandidates(QModelIndex index)
{
  ui->checkBox_show_init_cameras->setChecked(false);
  //draw camera at selected position
  QModelIndexList sil = ui->tableView_scan_candidates->selectionModel()->selectedRows();
  vector<ScanCandidate> *seletedViewCameras  = area->dataMgr.getSelectedScanCandidates();
  vector<ScanCandidate> *allScanCandidates = area->dataMgr.getScanCandidates();

  seletedViewCameras->clear();
  //add selected scan candidates into selected vector
  for (int i = 0; i < sil.size(); ++i)
  {
    int row = sil[i].row();
    seletedViewCameras->push_back((*allScanCandidates)[row]);
  }
}

void CameraParaDlg::showSelectedScannedMesh(QModelIndex index)
{
  ui->checkBox_show_init_cameras->setChecked(false);
  //get selected rows
  QModelIndexList sil = ui->tableView_scan_results->selectionModel()->selectedRows();
  vector<CMesh* > *scanned_results = area->dataMgr.getScannedResults();

  int row_of_mesh = 0;
  vector<ScanCandidate> *seletedViewCameras  = area->dataMgr.getSelectedScanCandidates();
  vector<ScanCandidate> *allScanCandidates = area->dataMgr.getScanCandidates();
  seletedViewCameras->clear();

  for (vector<CMesh* >::iterator it = scanned_results->begin(); 
    it != scanned_results->end(); ++it, ++row_of_mesh)
  {
    if ((*it)->vert.empty()) continue;
    //set default to invisible
    (*it)->vert[0].is_scanned_visible = false;

    for (int i = 0; i < sil.size(); ++i)
    {
      int row = sil[i].row();
      //if the mesh is chosen, change to visible
      if (row == row_of_mesh) 
      {
        (*it)->vert[0].is_scanned_visible = true;
        seletedViewCameras->push_back((*allScanCandidates)[row]);
      }
    }
  }
}

void CameraParaDlg::mergeScannedMeshWithOriginal()
{
  /*QModelIndexList sil = ui->tableView_scan_results->selectionModel()->selectedRows();
  if (sil.isEmpty()) return;*/
  CMesh* original = area->dataMgr.getCurrentOriginal();
  vector<CMesh* > *scanned_results = area->dataMgr.getScannedResults();
  double merge_confidence_threshold = global_paraMgr.camera.getDouble("Merge Confidence Threshold");
  int merge_pow = static_cast<int>(global_paraMgr.nbv.getDouble("Merge Probability Pow"));
  double probability_add_by_user = 0.0;

  //wsh added 12-24
  double radius_threshold = global_paraMgr.data.getDouble("CGrid Radius");
  double radius2 = radius_threshold * radius_threshold;
  double iradius16 = -4/radius2;

  double sigma = global_paraMgr.norSmooth.getDouble("Sharpe Feature Bandwidth Sigma");
  double sigma_threshold = pow(max(1e-8,1-cos(sigma/180.0*3.1415926)), 2);
  CMesh* iso_points = area->dataMgr.getCurrentIsoPoints();
  //end wsh added

  /*int row_of_mesh = 0;*/
  for (vector<CMesh* >::iterator it = scanned_results->begin(); 
    it != scanned_results->end(); ++it /*++row_of_mesh*/)
  {
    if ((*it)->vert.empty()) continue;

    // for (int i = 0; i < sil.size(); ++i)
    // {
    //int row = sil[i].row();
    //combine the selected mesh with original
    // if (row == row_of_mesh) 
    // {
    //make the merged mesh invisible
    (*it)->vert[0].is_scanned_visible = false;
    //compute new scanned mesh's iso neighbors

    //wsh updated 12-24
    //GlobalFun::computeAnnNeigbhors(area->dataMgr.getCurrentIsoPoints()->vert, (*it)->vert, 1, false, "runNewScannedMeshNearestIsoPoint");
    Timer time;
    time.start("Sample ISOpoints Neighbor Tree!!");
    GlobalFun::computeBallNeighbors((*it), area->dataMgr.getCurrentIsoPoints(), 
      radius_threshold, (*it)->bbox);
    time.end();
    //end wsh updated

    cout<<"Before merge with original: " << original->vert.size() <<endl;
    cout<<"scanned mesh num: "<<(*it)->vert.size() <<endl;

    vector<double> v_confidence;
    double max_confidence = 0.0f;
    double min_confidence = BIG;

    for (int k = 0; k < (*it)->vert.size(); ++k)
    {
      //wsh updated 12-24
      CVertex& v = (*it)->vert[k];
      //add or not
      //CVertex &nearest = area->dataMgr.getCurrentIsoPoints()->vert[v.neighbors[0]];
      double sum_confidence = 0.0;
      double sum_w = 0.0;

      for(int ni = 0; ni < v.original_neighbors.size(); ni++)
      {
        CVertex& t = iso_points->vert[v.original_neighbors[ni]];

        double dist2 = GlobalFun::computeEulerDistSquare(v.P(), t.P());
        double dist_diff = exp(dist2 * iradius16);
        //double normal_diff = exp(-pow(1-v.N()*t.N(), 2)/sigma_threshold);
        double normal_diff = 1.0;
        double w = dist_diff * normal_diff;
        //double w = 1.0f;
        sum_confidence += w * t.eigen_confidence;
        sum_w += 1;
      }

      if (v.original_neighbors.size() > 0 )
        sum_confidence /= sum_w;

      v_confidence.push_back(sum_confidence);

      max_confidence = sum_confidence > max_confidence ? sum_confidence : max_confidence;
      min_confidence = sum_confidence < min_confidence ? sum_confidence : min_confidence;
    }

    //normalize the confidence
    for (vector<double>::iterator it = v_confidence.begin(); it != v_confidence.end(); ++it)
      *it = (*it - min_confidence) / (max_confidence - min_confidence);

    int skip_num = 0;
    int index = original->vert.empty() ? 0 : (original->vert.back().m_index + 1);
    for (int k = 0;  k < (*it)->vert.size(); ++k)
    {
      CVertex& v = (*it)->vert[k];
      if (/*v_confidence[k] > merge_confidence_threshold || */ 
        (1.0f * rand() / (RAND_MAX+1.0) > (pow((1.0 - v_confidence[k]), merge_pow) + probability_add_by_user))) //pow((1 - v_confidence[k]), merge_pow)
      {
        v.is_ignore = true;
        skip_num++; 
        continue;
      }

      CVertex new_v;
      new_v.m_index = index++;
      new_v.is_original = true;
      new_v.P() = v.P();
      new_v.N() = v.N();
      original->vert.push_back(new_v);
      original->bbox.Add(new_v.P());
    } 
    original->vn = original->vert.size();
    cout<<"skip points num:" <<skip_num <<endl;
    cout<<"After merge with original: " << original->vert.size() <<endl <<endl;
    //set combined row unable to be chosen
    //ui->tableView_scan_candidates->setRowHidden(row, true);
    //ui->tableView_scan_results->setRowHidden(row, true);
    //}
    // }//end for sil
  }
}

void CameraParaDlg::mergeScannedMeshWithOriginalUsingHoleConfidence()
{
  CMesh* original = area->dataMgr.getCurrentOriginal();
  CMesh* iso_points = area->dataMgr.getCurrentIsoPoints();
  assert(original != NULL);
  assert(iso_points != NULL);
  vector<CMesh* > *scanned_results = area->dataMgr.getScannedResults();
  for (vector<CMesh* >::iterator it = scanned_results->begin(); it != scanned_results->end(); ++it) 
  {
    if ((*it)->vert.empty())
      continue;

    GlobalFun::computeAnnNeigbhors(iso_points->vert, (*it)->vert, 1, false, "runComputeIsoSmoothnessConfidence");

    (*it)->vert[0].is_scanned_visible = false;
    cout<<"Before merge with original: " << original->vert.size() <<endl;
    cout<<"scanned mesh num: "<<(*it)->vert.size() <<endl;

    int skip_num = 0;
    int index = original->vert.empty() ? 0 : (original->vert.back().m_index + 1);
    const double skip_conf = 0.95f;
    for (int k = 0; k < (*it)->vert.size(); ++k)
    {
      CVertex& v = (*it)->vert[k];
      if(v.neighbors.empty())
        continue;

      double nei_confidence = iso_points->vert[v.neighbors[0]].eigen_confidence;
      if(nei_confidence > skip_conf){
        v.is_ignore = true;
        skip_num ++;
        continue;
      }

      CVertex new_v;
      new_v.m_index = index++;
      new_v.is_original = true;
      new_v.P() = v.P();
      new_v.N() = v.N();
      original->vert.push_back(new_v);
      original->bbox.Add(new_v.P());
    }
    original->vn = original->vert.size();
    cout<<"skip points num:" <<skip_num <<endl;
    cout<<"After merge with original: " << original->vert.size() <<endl <<endl;
  }
}

void CameraParaDlg::mergeScannedMeshWithOriginalByHand()
{
  QModelIndexList sil = ui->tableView_scan_results->selectionModel()->selectedRows();
  if (sil.isEmpty()) return;

  CMesh* original = area->dataMgr.getCurrentOriginal();
  vector<CMesh* > *scanned_results = area->dataMgr.getScannedResults();
  double merge_confidence_threshold = global_paraMgr.camera.getDouble("Merge Confidence Threshold");
  int merge_pow = static_cast<int>(global_paraMgr.nbv.getDouble("Merge Probability Pow"));
  double probability_add_by_user = 0.0;
  cout<< "merge_confidence_threshold: " <<merge_confidence_threshold <<endl;

  //wsh added 12-24
  double radius_threshold = global_paraMgr.data.getDouble("CGrid Radius");
  double radius2 = radius_threshold * radius_threshold;
  double iradius16 = -4/radius2;

  double sigma = global_paraMgr.norSmooth.getDouble("Sharpe Feature Bandwidth Sigma");
  double sigma_threshold = pow(max(1e-8,1-cos(sigma/180.0*3.1415926)), 2);
  CMesh* iso_points = area->dataMgr.getCurrentIsoPoints();
  //end wsh added

  int row_of_mesh = 0;
  for (vector<CMesh* >::iterator it = scanned_results->begin(); 
    it != scanned_results->end(); ++it, ++row_of_mesh)
  {
    if ((*it)->vert.empty()) continue;

    for (int i = 0; i < sil.size(); ++i)
    {
      int row = sil[i].row();
      //combine the selected mesh with original
      if (row == row_of_mesh) 
      {
        (*it)->vert[0].is_scanned_visible = false;
        //compute new scanned mesh's iso neighbors

        //wsh updated 12-24
        //GlobalFun::computeAnnNeigbhors(area->dataMgr.getCurrentIsoPoints()->vert, (*it)->vert, 1, false, "runNewScannedMeshNearestIsoPoint");
        Timer time;
        time.start("Sample ISOpoints Neighbor Tree!!");
        GlobalFun::computeBallNeighbors((*it), area->dataMgr.getCurrentIsoPoints(), 
          radius_threshold, (*it)->bbox);
        time.end();
        //end wsh updated

        cout<<"Before merge with original: " << original->vert.size() <<endl;
        cout<<"scanned mesh num: "<<(*it)->vert.size() <<endl;

        int skip_num = 0;

        vector<double> v_confidence;
        double max_confidence = 0.0f;
        double min_confidence = BIG;

        for (int k = 0; k < (*it)->vert.size(); ++k)
        {
          //wsh updated 12-24
          CVertex& v = (*it)->vert[k];
          //add or not
          //CVertex &nearest = area->dataMgr.getCurrentIsoPoints()->vert[v.neighbors[0]];
          double sum_confidence = 0.0;
          double sum_w = 0.0;

          for(int ni = 0; ni < v.original_neighbors.size(); ni++)
          {
            CVertex& t = iso_points->vert[v.original_neighbors[ni]];

            double dist2 = GlobalFun::computeEulerDistSquare(v.P(), t.P());
            double dist_diff = exp(dist2 * iradius16);
            //double normal_diff = exp(-pow(1-v.N()*t.N(), 2)/sigma_threshold);
            double normal_diff = 1.0;
            double w = dist_diff * normal_diff;
            //double w = 1.0f;
            sum_confidence += w * t.eigen_confidence;
            sum_w += 1;
          }

          if (v.original_neighbors.size() > 0 )
            sum_confidence /= sum_w;

          v_confidence.push_back(sum_confidence);

          max_confidence = sum_confidence > max_confidence ? sum_confidence : max_confidence;
          min_confidence = sum_confidence < min_confidence ? sum_confidence : min_confidence;
        }

        //normalize the confidence
        for (vector<double>::iterator it = v_confidence.begin(); it != v_confidence.end(); ++it)
          *it = (*it - min_confidence) / (max_confidence - min_confidence);

        int index = original->vert.empty() ? 0 : (original->vert.back().m_index + 1);
        for (int k = 0;  k < (*it)->vert.size(); ++k)
        {
          CVertex& v = (*it)->vert[k];

          if (k < 10)
            cout<<"sum_confidence: " << v_confidence[k] <<endl;

          if (/*v_confidence[k] > merge_confidence_threshold 
              || */(1.0f * rand() / (RAND_MAX+1.0) > (pow((1.0 - v_confidence[k]), merge_pow) + probability_add_by_user)))
          {
            v.is_ignore = true;
            skip_num++; 
            continue;
          }

          CVertex new_v;
          new_v.m_index = index++;
          new_v.is_original = true;
          new_v.P() = v.P();
          new_v.N() = v.N();
          new_v.C().SetRGB(255, 0, 0);
          original->vert.push_back(new_v);
          original->bbox.Add(new_v.P());
        } 
        original->vn = original->vert.size();
        cout<<"skip points num:" <<skip_num <<endl;
        cout<<"After merge with original: " << original->vert.size() <<endl <<endl;
        //set combined row unable to be chosen
        //ui->tableView_scan_candidates->setRowHidden(row, true);
        //ui->tableView_scan_results->setRowHidden(row, true);
      }
    }
  }
}

void CameraParaDlg::getNbvIterationCount(int _val)
{
  global_paraMgr.nbv.setValue("NBV Iteration Count", IntValue(_val));
}

void CameraParaDlg::getNBVTopN(int _val)
{
  global_paraMgr.nbv.setValue("NBV Top N", IntValue(_val));
}

void CameraParaDlg::getNBVConfidenceThreshold(double _val)
{
  global_paraMgr.nbv.setValue("View Prune Confidence Threshold", DoubleValue(_val));
}

void CameraParaDlg::getCameraHorizonDist(double _val)
{
  global_paraMgr.camera.setValue("Camera Horizon Dist", DoubleValue(_val));
}

void CameraParaDlg::getCameraVerticalDist(double _val)
{
  global_paraMgr.camera.setValue("Camera Vertical Dist", DoubleValue(_val));
}

void CameraParaDlg::getCameraDistToModel(double _val)
{
  global_paraMgr.camera.setValue("Camera Dist To Model", DoubleValue(_val));
}

void CameraParaDlg::getCameraResolution(int _val)
{
  global_paraMgr.camera.setValue("Camera Resolution", DoubleValue(1.0f / _val));
}

void CameraParaDlg::getViewPruneConfidenceThreshold(double _val)
{
  global_paraMgr.nbv.setValue("View Prune Confidence Threshold", DoubleValue(_val));
}

void CameraParaDlg::getGridResolution(double _val)
{
  global_paraMgr.nbv.setValue("View Grid Resolution", DoubleValue(_val));
}

void CameraParaDlg::getMaxRaySteps(double _val)
{
  global_paraMgr.nbv.setValue("Max Ray Steps Para", DoubleValue(_val));
}

void CameraParaDlg::getIsoBottomDelta(double _val)
{
  global_paraMgr.nbv.setValue("Iso Bottom Delta", DoubleValue(_val));
}
void CameraParaDlg::getMergeConfidenceThreshold(double _val)
{
  global_paraMgr.camera.setValue("Merge Confidence Threshold", DoubleValue(_val));
}

void CameraParaDlg::getCameraFarDistance(double _val)
{
  global_paraMgr.camera.setValue("Camera Far Distance", DoubleValue(_val));
}

void CameraParaDlg::getCameraNearDistance(double _val)
{
  global_paraMgr.camera.setValue("Camera Near Distance", DoubleValue(_val));
}

void CameraParaDlg::getPredictedModelSize(double _val)
{
  global_paraMgr.camera.setValue("Predicted Model Size", DoubleValue(_val));
}

void CameraParaDlg::getOptimalPlaneWidth(double _val)
{
  global_paraMgr.camera.setValue("Optimal Plane Width", DoubleValue(_val));
}

void CameraParaDlg::getPropagateIndex(double _val)
{
  global_paraMgr.nbv.setValue("Propagate One Point Index", DoubleValue(_val));
}

void CameraParaDlg::getRayResolutionPara(double _val)
{
  global_paraMgr.nbv.setValue("Ray Resolution Para", DoubleValue(_val));
  //global_paraMgr.nbv.setValue("Max Displacement", DoubleValue(_val));
}

void CameraParaDlg::buildGrid()
{
  global_paraMgr.nbv.setValue("Run Build Grid", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Build Grid", BoolValue(false));
}

void CameraParaDlg::propagate()
{
  global_paraMgr.nbv.setValue("Run Propagate", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Propagate", BoolValue(false));
}

void CameraParaDlg::propagateOnePoint()
{
  global_paraMgr.nbv.setValue("Run Propagate", BoolValue(true));
  global_paraMgr.nbv.setValue("Run Propagate One Point", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Propagate One Point", BoolValue(false));
  global_paraMgr.nbv.setValue("Run Propagate", BoolValue(false));
}

void CameraParaDlg::gridSegment()
{
  global_paraMgr.nbv.setValue("Run Build Grid", BoolValue(true));
  global_paraMgr.nbv.setValue("Run Grid Segment", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Grid Segment", BoolValue(false));
  global_paraMgr.nbv.setValue("Run Build Grid", BoolValue(false));

}

void CameraParaDlg::extractViewCandidates()
{
  global_paraMgr.nbv.setValue("Run Viewing Extract", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Viewing Extract", BoolValue(false));
}

void CameraParaDlg::extractViewIntoBins()
{
  global_paraMgr.nbv.setValue("Run Extract Views Into Bins", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Extract Views Into Bins", BoolValue(false));
}

void CameraParaDlg::runViewClustering()
{
  global_paraMgr.nbv.setValue("Run Viewing Clustering", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Viewing Clustering", BoolValue(false));
}

void CameraParaDlg::runViewPrune()
{
  global_paraMgr.nbv.setValue("Run View Prune", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run View Prune", BoolValue(false));
  updateTableViewNBVCandidate();
}

void CameraParaDlg::runSetIsoBottomConfidence()
{
  global_paraMgr.nbv.setValue("Run Set Iso Bottom Confidence", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Set Iso Bottom Confidence", BoolValue(false));
}

void CameraParaDlg::runUpdateViewDirections()
{
  global_paraMgr.nbv.setValue("Run Update View Directions", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Update View Directions", BoolValue(false));
}

void CameraParaDlg::runSetupInitialScanns()
{
  global_paraMgr.camera.setValue("Run Initial Scan", BoolValue(true));
  area->runCamera();
  global_paraMgr.camera.setValue("Run Initial Scan", BoolValue(false));
  updateTableViewNBVCandidate();
  //get initial radius
  area->dataMgr.downSamplesByNum();
  area->updateGL();
}

void CameraParaDlg::runStep2CombinedPoissonConfidence()
{
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(true));
  global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(true));
  global_paraMgr.poisson.setValue("Run One Key PoissonConfidence", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(false));
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(false));
  global_paraMgr.poisson.setValue("Run One Key PoissonConfidence", BoolValue(false));
}

void CameraParaDlg::runStep2HolePoissonConfidence()
{
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(true));
  global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(true));
  global_paraMgr.poisson.setValue("Compute Hole Confidence", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(false));
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(false));
  global_paraMgr.poisson.setValue("Compute Hole Confidence", BoolValue(false));
}

void CameraParaDlg::runStep2PoissonConfidenceViaOriginal()
{
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(true));
  global_paraMgr.poisson.setValue("Run One Key PoissonConfidence", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run One Key PoissonConfidence", BoolValue(false));
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(false));
}

//confidence is computed always on iso points
void CameraParaDlg::runSceneConfidence()
{
  global_paraMgr.poisson.setValue("Run Scene Confidence", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Scene Confidence", BoolValue(false));
}

//force run poisson reconstruction on original points
void CameraParaDlg::runSceneConfidenceViaOriginal()
{
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(true));
  global_paraMgr.poisson.setValue("Run Scene Confidence", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Scene Confidence", BoolValue(false));
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(false));
}

void CameraParaDlg::runStep3NBVcandidates()
{
  global_paraMgr.nbv.setValue("Run One Key NBV", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run One Key NBV", BoolValue(false));
  updateTableViewNBVCandidate();
  ui->tableView_scan_results->clearSpans();
}

void CameraParaDlg::runStep4NewScans()
{
  global_paraMgr.camera.setValue("Run One Key NewScans", BoolValue(true));
  area->runCamera();
  global_paraMgr.camera.setValue("Run One Key NewScans", BoolValue(false));
  updateTabelViewScanResults();
}

//put it in the other thread
void CameraParaDlg::runOneKeyNbvIterationBack()
{
  QString file_location = QFileDialog::getExistingDirectory(this, "choose a directory...", "",QFileDialog::ShowDirsOnly);
  if (!file_location.size()) return;

  QThread *thread = new QThread();
  m_nbv.moveToThread(thread);
  QObject::connect(this, SIGNAL(sig_runOneKeyNbvIterationBack(QString, GLArea*)), &m_nbv, SLOT(oneKeyNBV(QString, GLArea*)));
  QObject::connect(&m_nbv, SIGNAL(updateTableViewNBVCandidate()), this, SLOT(updateTableViewNBVCandidate()));
  QObject::connect(&m_nbv, SIGNAL(updateTabelViewScanResults()), this, SLOT(updateTabelViewScanResults()));
  QObject::connect(&m_nbv, SIGNAL(mergeScannedMeshWithOriginal()), this, SLOT(mergeScannedMeshWithOriginal()));

  thread->start();
  printf("main thread: %d\n", QThread::currentThreadId());
  emit sig_runOneKeyNbvIterationBack(file_location, area);
  //we can't add wait here, wait means this function can not return before the thread ends
}

void CameraParaDlg::runOneKeyNbvIteration()
{
  QString file_location = QFileDialog::getExistingDirectory(this, "choose a directory...", "",QFileDialog::ShowDirsOnly);
  if (!file_location.size()) return;

  QString s_log = "\\log.txt";
  s_log = file_location + s_log;
  ofstream log;
  log.open(s_log.toAscii().data());
  cout.rdbuf(log.rdbuf());

  QString para = "\\parameter.para";
  para = file_location + para;
  area->dataMgr.saveParameters(para);

  int iteration_cout = global_paraMgr.nbv.getInt("NBV Iteration Count");
  const int holeFrequence = 3; //once every holeFrequence(2, 3, ...)
  bool use_hole_confidence = false;
  CMesh *original = area->dataMgr.getCurrentOriginal();

  for (int ic = 0; ic < iteration_cout; ++ic)
  {
    if (ic % holeFrequence == 0){
      use_hole_confidence = true;
    }else{
      use_hole_confidence = false;
    }
    //save original
    QString s_original;
    s_original.sprintf("\\%d_original.ply", ic);
    s_original = file_location + s_original;
    area->dataMgr.savePly(s_original, *area->dataMgr.getCurrentOriginal());

    //compute normal on original
    vector<Point3f> before_normal;
    for (int i = 0; i < original->vert.size(); ++i)
      before_normal.push_back(original->vert[i].N()); 

    int knn = global_paraMgr.norSmooth.getInt("PCA KNN");
    vcg::tri::PointCloudNormal<CMesh>::Param pca_para;
    pca_para.fittingAdjNum = knn;
    //fixme: a debug error in compute
    vcg::tri::PointCloudNormal<CMesh>::Compute(*original, pca_para, NULL);

    for (int i = 0; i < original->vert.size(); ++i)
    {
      if (before_normal[i] * original->vert[i].N() < 0.0f)
        original->vert[i].N() *= -1;
    }

    //save normalized original
    QString s_normal_original;
    s_normal_original.sprintf("\\%d_normal_original.ply", ic);
    s_normal_original = file_location + s_normal_original;
    area->dataMgr.savePly(s_normal_original, *area->dataMgr.getCurrentOriginal());

    //compute radius
    area->dataMgr.downSamplesByNum();
    area->initSetting();

    if(use_hole_confidence)
    {
      cout<<"begin to run hole poisson confidence" <<endl;
      runStep2HolePoissonConfidence();
      cout<<"begin to run hole poisson confidence" <<endl;
    }else{
      cout<<"begin to run combined poisson confidence" <<endl;
      runStep2CombinedPoissonConfidence();
      cout<<"end run combined poisson confidence" <<endl;
    }
    //save poisson surface "copy poisson_out.ply file_location\\%d_poisson_out.ply"
    cout<<"begin to copy poisson_surface" <<endl;
    QString s_poisson_surface;
    s_poisson_surface.sprintf("\\%d_poisson_out.ply", ic);
    QString s_cmd_copy_poisson = "copy poisson_out.ply ";
    s_cmd_copy_poisson += file_location;
    s_cmd_copy_poisson += s_poisson_surface;
    cout << s_cmd_copy_poisson.toStdString() <<endl;
    system(s_cmd_copy_poisson.toAscii().data());
    cout<<"end to copy poisson_surface" <<endl;
    //save iso-skel and view
    QString s_iso;
    s_iso.sprintf("\\%d_iso.skel", ic);
    s_iso = file_location + s_iso;
    s_iso.replace(".skel", ".View");
    area->saveView(s_iso);
    //save iso dat and raw
    s_iso.replace(".View", ".raw");
    global_paraMgr.poisson.setValue("Run Normalize Field Confidence", BoolValue(true));  
    area->runPoisson();
    global_paraMgr.poisson.setValue("Run Normalize Field Confidence", BoolValue(false));  
    area->dataMgr.saveFieldPoints(s_iso);    

    runStep3NBVcandidates();
    NBVCandidatesScan();
    //save nbv skel and view
    QString s_nbv;
    s_nbv.sprintf("\\%d_nbv.skel", ic);
    s_nbv = file_location + s_nbv;
    s_nbv.replace(".skel", ".View");
    area->saveView(s_nbv);

    if (use_hole_confidence){
      mergeScannedMeshWithOriginalUsingHoleConfidence();
    }else{
      mergeScannedMeshWithOriginal();
    }
    //save merged scan
    cout<<"begin to save merged mesh" <<endl;
    QString s_merged_mesh;
    s_merged_mesh.sprintf("\\%d_merged_mesh", ic);
    s_merged_mesh = file_location + s_merged_mesh;
    area->dataMgr.saveMergedMesh(s_merged_mesh);
    cout<< "end save merged mesh" <<endl;

    /* cout << "Begin remove outliers!" <<endl;
    GlobalFun::removeOutliers(original, global_paraMgr.data.getDouble("CGrid Radius") * 2, 10);
    cout << "End remove outliers!" <<endl;*/
  }

  QString last_original = "\\ultimate_original.ply";
  last_original = file_location + last_original;
  area->dataMgr.savePly(last_original, *area->dataMgr.getCurrentOriginal());
  log.close();
}

void CameraParaDlg::getModelSize()
{
  CMesh* original = area->dataMgr.getCurrentOriginal();
  original->bbox.SetNull();
  for (int i = 0; i < original->vert.size(); i++)
  {
    original->bbox.Add(original->vert[i]);
  }

  Box3f box = original->bbox;

  GlobalFun::printPoint3(cout, box.min);
  GlobalFun::printPoint3(cout, box.max);

  float dist1 = abs(box.min.X() - box.max.X());
  float dist2 = abs(box.min.Y() - box.max.Y());
  float dist3 = abs(box.min.Z() - box.max.Z());

  float max_dist = dist1 > dist2 ? dist1 : dist2;
  max_dist = max_dist > dist3 ? max_dist : dist3;


  m_paras->camera.setValue("Predicted Model Size", DoubleValue(max_dist/10.));

  initWidgets();
  update();

  //m_paras->camera.getDouble("Predicted Model Size")
}

void CameraParaDlg::loadSDFVoxels()
{
  QString file = QFileDialog::getOpenFileName(this, "Select a ply file", "", "*.ply");
  if(!file.size()) return;

  area->dataMgr.loadOwnToSDFVoxelBinary(file);
  std::cout<<"SDF Voxel points num: " <<area->dataMgr.sdf_voxels.vert.size() <<std::endl;

  //initAfterOpenFile();
  area->dataMgr.getInitRadiuse();
  //area->dataMgr.normalizeAllMesh();

  double after_normalize = global_paraMgr.nbv.getDouble("SDF Voxel Size") / global_paraMgr.data.getDouble("Max Normalize Length");
  global_paraMgr.nbv.setValue("SDF Voxel Size", DoubleValue(after_normalize));

  /*for (int i = 0; i < area->dataMgr.sdf_voxels.vert.size(); ++i)
  {
  printf("x: %lf, y: %lf, z: %lf\n", 
  area->dataMgr.sdf_voxels.vert[i].P()[0], area->dataMgr.sdf_voxels.vert[i].P()[1], area->dataMgr.sdf_voxels.vert[i].P()[2]);
  }*/

  prepareSDFSlicePlane();
}

void CameraParaDlg::runSDFSlice()
{
  global_paraMgr.nbv.setValue("Run SDF Slice", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run SDF Slice", BoolValue(false));
}

void CameraParaDlg::prepareSDFSlicePlane()
{
  Point3f b_max = area->dataMgr.sdf_voxels.bbox.max;
  Point3f b_min = area->dataMgr.sdf_voxels.bbox.min;
  cout<<"bbox min: " <<b_min[0] <<" " <<b_min[1] <<" " <<b_min[2] <<endl;
  cout<<"bbox max: " <<b_max[0] <<" " <<b_max[1] <<" " <<b_max[2] <<endl;

  double max_edge = max(max(b_max.X() - b_min.X(), b_max.Y() - b_min.Y()), b_max.Z() - b_min.Z());
  cout<<"sdf voxels max edge: " << max_edge <<endl;

  double sdf_cell_size = global_paraMgr.nbv.getDouble("SDF Voxel Size");
  int res = max_edge / sdf_cell_size;
  cout<<"sdf slice plane resolution: " << res <<endl;
  global_paraMgr.nbv.setValue("SDF Slice Plane Resolution", IntValue(res));

  //generate x axis slice plane
  int x_idx = 0;
  for(int k = 0; k < res; ++k) for(int j = 0; j < res; ++j){
    CVertex v_x;
    v_x.m_index = x_idx++;
    v_x.is_sdf = true;
    v_x.P()[0] = 0.0f;
    v_x.P()[1] = b_min.Y() + j * sdf_cell_size;
    v_x.P()[2] = b_min.Z() + k * sdf_cell_size;
    area->dataMgr.x_sdf_slice_plane.vert.push_back(v_x);
    area->dataMgr.x_sdf_slice_plane.bbox.Add(v_x.P());
  }
  area->dataMgr.x_sdf_slice_plane.vn = area->dataMgr.x_sdf_slice_plane.vert.size();

  //generate y axis slice plane
  int y_idx = 0;
  for(int k = 0; k < res; ++k) for(int i = 0; i < res; ++i){
    CVertex v_y;
    v_y.m_index = y_idx++;
    v_y.is_sdf = true;
    v_y.P()[0] = b_min.X() + i * sdf_cell_size;
    v_y.P()[1] = 0.0f;
    v_y.P()[2] = b_min.Z() + k * sdf_cell_size;
    area->dataMgr.y_sdf_slice_plane.vert.push_back(v_y);
    area->dataMgr.y_sdf_slice_plane.bbox.Add(v_y.P());
  }
  area->dataMgr.y_sdf_slice_plane.vn = area->dataMgr.y_sdf_slice_plane.vert.size();

  //generate z axis slice plane
  int z_idx = 0;
  for(int j = 0; j < res; ++j) for( int i = 0; i < res; ++i){
    CVertex v_z;
    v_z.m_index = z_idx++;
    v_z.is_sdf = true;
    v_z.P()[0] = b_min.X() + i * sdf_cell_size;
    v_z.P()[1] = b_min.Y() + j * sdf_cell_size;
    v_z.P()[2] = 0.0f;
    area->dataMgr.z_sdf_slice_plane.vert.push_back(v_z);
    area->dataMgr.z_sdf_slice_plane.bbox.Add(v_z);
  }
  area->dataMgr.z_sdf_slice_plane.vn = area->dataMgr.z_sdf_slice_plane.vert.size();

  cout<<"prepare sdf slice plane done!" <<endl;
}


void CameraParaDlg::loadScene()
{
  unsigned short color = 2;
  int last = 0;//chunkDesc->m_SDFBlocks[i].data[2 * j + 1];
  last &= 0xffff00ff;           //先清空颜色的r分量
  color &= 0x000000ff;      //取出颜色的bit位
  last |= (color << 0x8);            //更新颜色的r分量
  std::cout<<((last & 0x0000ff00) >> 0x8) <<std::endl;
  return;

  //load scene to original
<<<<<<< .mine
	int x = 0x7fffffff;
	std::cout<<x <<std::endl;
	std::cout<<INT_MAX <<std::endl;
	return;
	loadToOriginal();














=======
  ifstream color_in("color_in.txt");
  ofstream color_out("color_out.txt");
  string line;
  int r, g, b;
  while(getline(color_in, line)){
    stringstream ss(line);
    ss>>r >>g >>b;
    char tmp[50];
    sprintf(tmp, "float4(%.2ff, %.2ff, %.2ff, 0.0f),\n", r / 255.0f, g / 255.0f, b / 255.0f);
    color_out<<tmp;
  }

  color_in.close();
  color_out.close();
  return;



  loadToOriginal();
>>>>>>> .theirs
}

void CameraParaDlg::detectPlane()
{
  std::cout<<"detect plane" <<endl;
  CMesh *original = area->dataMgr.getCurrentOriginal();
  if (original == NULL) {
    std::cout<<"original point NULL, No Plane Detected!" <<std::endl;
    return;
  }

  pcl::SACSegmentation<PclPoint> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.02);

  PclPointCloudPtr original_point_cloud(new PclPointCloud);

  GlobalFun::CMesh2PclPointCloud(original, original_point_cloud);
  seg.setInputCloud(original_point_cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size () == 0) {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    return;
  }else{
    std::cout<< "coefficients:" <<  *coefficients <<std::endl;
  }
  return;
}

void CameraParaDlg::computeSceneConfidence()
{
  //only to generate field points
  global_paraMgr.poisson.setValue("Run Poisson On Samples", BoolValue(true));
  global_paraMgr.poisson.setValue("Run Generate Poisson Field", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Poisson On Samples", BoolValue(false));
  global_paraMgr.poisson.setValue("Run Generate Poisson Field", BoolValue(false));

  //compute scene confidence on iso points default
  runSceneConfidence();

  area->updateUI();
  area->updateGL();
}

void CameraParaDlg::computeSceneNBV()
{
  runStep3NBVcandidates();
};

void CameraParaDlg::savePickPointToIso()
{
  area->savePickPointToIso();
  area->cleanPickPoints();
}

void CameraParaDlg::detectChangedPoints()
{

}

void CameraParaDlg::updateShowModel()
{
	/////////////////////////////////////show the results
	CMesh *graphCutResult_mesh = area->dataMgr.getCurrentGraphCutResult();
	GRAPHSHOW *contractionGraph = area->dataMgr.getContractionGraph();
	GRAPHSHOW *patchGraph = area->dataMgr.getPatchGraph();

	srand((unsigned)time(0));
	graphCutResult_mesh->Clear();
	for(int i = 0; i < cPointCloudAnalysis.cScanEstimation.vecObjectHypo.size();i++)
	{
		if(cPointCloudAnalysis.cScanEstimation.vecObjectHypo[i].mergeFlag)	continue;

		double r,g,b;
		r = double(rand()%255);
		g = double(rand()%255);
		b = double(rand()%255);

		for(int j = 0; j < cPointCloudAnalysis.cScanEstimation.vecObjectHypo[i].patchIndex.size();j++)
		{
			int patchIndex = cPointCloudAnalysis.cScanEstimation.vecObjectHypo[i].patchIndex[j];
			for(int k = 0; k < cPointCloudAnalysis.cMultiSeg.vecPatchPoint[patchIndex].mypoints.size();k++)
			{
				MyPoint_RGB_NORMAL point = cPointCloudAnalysis.cMultiSeg.vecPatchPoint[patchIndex].mypoints[k];
				CVertex new_point;
				new_point.P()[0] = point.x - (cPointCloudAnalysis.cBinarySeg.xMax + cPointCloudAnalysis.cBinarySeg.xMin)/2;
				new_point.P()[1] = point.y - (cPointCloudAnalysis.cBinarySeg.yMax + cPointCloudAnalysis.cBinarySeg.yMin)/2;
				new_point.P()[2] = point.z - (cPointCloudAnalysis.cBinarySeg.zMax + cPointCloudAnalysis.cBinarySeg.zMin)/2;
				new_point.N()[0] = point.normal_x;
				new_point.N()[1] = point.normal_y;
				new_point.N()[2] = point.normal_z;

				new_point.C()[0] = r;
				new_point.C()[1] = g;
				new_point.C()[2] = b;

				new_point.m_index = i;
				new_point.is_graphcut_related = true;
				graphCutResult_mesh->vert.push_back(new_point);
				graphCutResult_mesh->bbox.Add(new_point.P());
			}
		}
	}

	for(int i = 0; i < cPointCloudAnalysis.cBinarySeg.tablePoint.mypoints.size();i++)
	{
		MyPoint_RGB_NORMAL point = cPointCloudAnalysis.cBinarySeg.tablePoint.mypoints[i];
		CVertex new_point;
		new_point.P()[0] = point.x - (cPointCloudAnalysis.cBinarySeg.xMax + cPointCloudAnalysis.cBinarySeg.xMin)/2;;
		new_point.P()[1] = point.y - (cPointCloudAnalysis.cBinarySeg.yMax + cPointCloudAnalysis.cBinarySeg.yMin)/2;
		new_point.P()[2] = point.z - (cPointCloudAnalysis.cBinarySeg.zMax + cPointCloudAnalysis.cBinarySeg.zMin)/2;
		new_point.N()[0] = 0;
		new_point.N()[1] = 0;
		new_point.N()[2] = 1;

		new_point.C()[0] = 0;
		new_point.C()[1] = 0;
		new_point.C()[2] = 0;

		new_point.m_index = i;
		new_point.is_graphcut_related = true;
		graphCutResult_mesh->vert.push_back(new_point);
		graphCutResult_mesh->bbox.Add(new_point.P());
	}
	graphCutResult_mesh->vn = graphCutResult_mesh->vert.size();

// 	contractionGraph->vecEdgeColor = cPointCloudAnalysis.cScanEstimation.graphContract.vecEdgeColor;
// 	contractionGraph->vecEdges = cPointCloudAnalysis.cScanEstimation.graphContract.vecEdges;
// 	contractionGraph->vecNodes = cPointCloudAnalysis.cScanEstimation.graphContract.vecNodes;
// 	contractionGraph->vecEdgeFlag = cPointCloudAnalysis.cScanEstimation.graphContract.vecEdgeFlag;
// 	contractionGraph->vecNodeFlag = cPointCloudAnalysis.cScanEstimation.graphContract.vecNodeFlag;

	contractionGraph->vecEdgeColor = cPointCloudAnalysis.cMultiSeg.graphContract.vecEdgeColor;
	contractionGraph->vecEdges = cPointCloudAnalysis.cMultiSeg.graphContract.vecEdges;
	contractionGraph->vecNodes = cPointCloudAnalysis.cMultiSeg.graphContract.vecNodes;
	contractionGraph->vecEdgeFlag = cPointCloudAnalysis.cMultiSeg.graphContract.vecEdgeFlag;
	contractionGraph->vecNodeFlag = cPointCloudAnalysis.cMultiSeg.graphContract.vecNodeFlag;

	patchGraph->vecEdgeColor = cPointCloudAnalysis.cBinarySeg.graphInit.vecEdgeColor;
	patchGraph->vecEdges = cPointCloudAnalysis.cBinarySeg.graphInit.vecEdges;
	patchGraph->vecNodes = cPointCloudAnalysis.cBinarySeg.graphInit.vecNodes;
}

void CameraParaDlg::runGraphCut()
{
	std::cout<<"test graph cut" <<std::endl;	

	ofstream outFileg("Output\\RunGraphCut.txt");
	outFileg << "let's begin :) " << endl;

	/////////////////////////////////////run graph cut
	cPointCloudAnalysis.MainStep(true,0);
	outFileg << "MainStep finished :)   " << endl;

	/////////////////////////////////////compute the score,sorting
	CMesh *original = area->dataMgr.getCurrentOriginal();
	for(int i = 0;i < cPointCloudAnalysis.cMultiSeg.vecvecMultiResult.size();i++)
	{
		cout<<"iteration:"<<i<<endl;
		GlobalFun::clearCMesh(*original);
		cPointCloudAnalysis.cScanEstimation.saveMultiResultToOriginal(original, i);
		area->dataMgr.downSamplesByNum();
		global_paraMgr.data.setValue("CGrid Radius", DoubleValue(0.036));
		runStep2CombinedPoissonConfidence();

		CMesh *iso_point = area->dataMgr.getCurrentIsoPoints();
		OBJECTISOPOINT objectIsoPointTemp;
		for(int j = 0;j < iso_point->vert.size();j++)
		{
			ISOPOINT isoPointTemp;
			isoPointTemp.f = iso_point->vert[j].eigen_confidence;
			isoPointTemp.objectIndex = j;
			isoPointTemp.x = iso_point->vert[j].P()[0];
			isoPointTemp.y = iso_point->vert[j].P()[1];
			isoPointTemp.z = iso_point->vert[j].P()[2];
			objectIsoPointTemp.objectIsoPoint.push_back(isoPointTemp);
		}
		cPointCloudAnalysis.cScanEstimation.vecObjectIsoPoint.push_back(objectIsoPointTemp);
	}
	outFileg << "Poisson finished :)   " << endl;

	cPointCloudAnalysis.cScanEstimation.ScoreUpdate();
	outFileg << "ScoreUpdate finished :)   " << endl;

	/////////////////////////////////////show result
	updateShowModel();
	outFileg << "Show results finished :)   " << endl;

	/////////////////////////////////////action according to the score
	//输出点和方向

	outFileg << "First round finished :) " << endl;
	outFileg.close();


// 	CMesh moving_mesh; 
// 	CMesh *static_mesh;

// 	int mask= tri::io::Mask::IOM_VERTNORMAL;
// 	int err = tri::io::Importer<CMesh>::Open(moving_mesh, "diff_moving.ply",mask);  
// 	outFile1 << "get moving mesh  " << endl;
//  
// 	int begin = cPointCloudAnalysis.cScanEstimation.clusterPatchInitIndex[pushArea];
// 	int end = cPointCloudAnalysis.cScanEstimation.clusterPatchInitIndex[pushArea] + cPointCloudAnalysis.cScanEstimation.clusterPatchNum[pushArea];
// 	for(int i = begin;i < end;i++)
// 	{
// 		for(int k = 0; k < cPointCloudAnalysis.cBinarySeg.vecPatchPoint[i].mypoints.size();k++)
// 		{
// 			MyPoint_RGB_NORMAL point = cPointCloudAnalysis.cBinarySeg.vecPatchPoint[i].mypoints[k];
// 			CVertex new_point;
// 			new_point.P()[0] = point.x;
// 			new_point.P()[1] = point.y;
// 			new_point.P()[2] = point.z;
// 			new_point.N()[0] = point.normal_x;
// 			new_point.N()[1] = point.normal_y;
// 			new_point.N()[2] = point.normal_z;
// 
// 			new_point.m_index = i;
// 			new_point.is_original = true;
// 			static_mesh->vert.push_back(new_point);
// 			static_mesh->bbox.Add(new_point.P());
// 		}
// 	}
// 	static_mesh->vn = static_mesh->vert.size();
// 	outFile1 << "get static mesh  " << endl;
// 
// 	double error;
// 	GlobalFun::computeICPNoNormal(moving_mesh, static_mesh, error);
// 	outFile1 << "get error:  " << error << endl;
// 
// 	if(error < 0.005)
// 		separateFlag = false;
// 	else 
// 		separateFlag = true;
}

void CameraParaDlg::updateGraphCut()
{
	std::cout<<"update graph cut" <<std::endl;

	ofstream outFileg("Output\\UpdateGraphCut.txt");
	outFileg << "let's begin :) " << endl;

	/////////////////////////////////////get new cloud and separateFlag
	int pushArea = 0;
	bool separateFlag;
	separateFlag = true;

	/////////////////////////////////////recompute
	int newAreaNum = 0;
	if(!separateFlag)	
	{
		outFileg << "Merge start:) " << endl;
		cPointCloudAnalysis.Merge(pushArea);
		outFileg << "Merge finished  :) " << endl;
	}
	else
	{
		outFileg << "ReAnalysis start:) " << endl;
		cPointCloudAnalysis.ReAnalysis(pushArea);

		outFileg << "Get new data :) " << endl;
		//over-segmentation again
		newAreaNum += getUpdateData();

		outFileg << "Begin Again :) " << endl;
		cPointCloudAnalysis.MainStep(false,newAreaNum);
	}

	CMesh *original = area->dataMgr.getCurrentOriginal();
	for(int i = 0;i < cPointCloudAnalysis.cMultiSeg.vecvecMultiResult.size();i++)
	{
		cout<<"iteration:"<<i<<endl;
		GlobalFun::clearCMesh(*original);
		cPointCloudAnalysis.cScanEstimation.saveMultiResultToOriginal(original, i);
		area->dataMgr.downSamplesByNum();
		global_paraMgr.data.setValue("CGrid Radius", DoubleValue(0.036));
		runStep2CombinedPoissonConfidence();

		CMesh *iso_point = area->dataMgr.getCurrentIsoPoints();
		OBJECTISOPOINT objectIsoPointTemp;
		for(int j = 0;j < iso_point->vert.size();j++)
		{
			ISOPOINT isoPointTemp;
			isoPointTemp.f = iso_point->vert[j].eigen_confidence;
			isoPointTemp.objectIndex = j;
			isoPointTemp.x = iso_point->vert[j].P()[0];
			isoPointTemp.y = iso_point->vert[j].P()[1];
			isoPointTemp.z = iso_point->vert[j].P()[2];
			objectIsoPointTemp.objectIsoPoint.push_back(isoPointTemp);
		}
		cPointCloudAnalysis.cScanEstimation.vecObjectIsoPoint.push_back(objectIsoPointTemp);
	}
	outFileg << "Poisson finished :)   " << endl;

	cPointCloudAnalysis.cScanEstimation.ScoreUpdate();
	outFileg << "ScoreUpdate finished :)   " << endl;

	updateShowModel();
	outFileg << "Show results finished :)   " << endl;

	outFileg.close();
}

int CameraParaDlg::getUpdateData()
{
  Visualizer vs;
  vs.viewer->removeAllPointClouds();
  vs.viewer->removeCoordinateSystem();
  vs.viewer->setBackgroundColor(0,0,0);

  PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);
  loadPointCloud_normal_ply("data/1.ply", cloud);

  /******************detect table************************/
  PointCloudPtr_RGB_NORMAL tabletopCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr_RGB_NORMAL planeCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr rect_cloud(new PointCloud());
  PointCloudPtr_RGB_NORMAL remainingCloud(new PointCloud_RGB_NORMAL());
  pcl::ModelCoefficients coefficients;

//   detect_table(cloud, coefficients, planeCloud, rect_cloud, remainingCloud);
// 
//   PointCloudPtr_RGB pc(new PointCloud_RGB);
// 
//   for(int i=0;i<planeCloud->size();i++){
//     Point_RGB pr;
//     pr.x=planeCloud->at(i).x;
//     pr.y=planeCloud->at(i).y;
//     pr.z=planeCloud->at(i).z;
//     pr.r=planeCloud->at(i).r;
//     pr.g=planeCloud->at(i).g;
//     pr.b=planeCloud->at(i).b;
//     pc->push_back(pr);
//   }
// 
//   vs.viewer->addPointCloud (pc, "table_cloud");
// 
// 
//   Eigen::Matrix4f matrix_transform;
//   Eigen::Matrix4f matrix_transform_r;
// 
//   getTemTransformMatrix(coefficients, matrix_transform, matrix_transform_r);
// 
//   getCloudOnTable(remainingCloud, rect_cloud, matrix_transform, matrix_transform_r, tabletopCloud);

  float voxel_resolution = 0.004f;
  float seed_resolution = 0.06f;
  float color_importance = 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  /******************Euclidean Cluster Extraction************************/
  std::vector<MyPointCloud_RGB_NORMAL> cluster_points;

  object_seg_ECE(cloud, cluster_points);

/*  std::cout<<"cloud num: " << cloud-.size() << std::endl;*/
  std::cout<<"cluster_points num: " << cluster_points.size() << std::endl;

  for(int i=0;i<cluster_points.size();i++){
    if(cluster_points.at(i).mypoints.size()<200){
      continue;
    }

    PointCloudT::Ptr colored_cloud(new PointCloudT);
    vector<MyPointCloud_RGB_NORMAL> patch_clouds;
    PointNCloudT::Ptr normal_cloud(new PointNCloudT);

    PointCloudPtr_RGB_NORMAL ct(new PointCloud_RGB_NORMAL);
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(cluster_points.at(i), ct);

    VCCS_over_segmentation(ct,voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance,patch_clouds,colored_cloud,normal_cloud);

		std::stringstream str;
		str<<"colored_voxel_cloud"<<i;
		std::string id_pc=str.str();

		

		//add normal, point cloud, cluster patch num
		for(int i=0;i<patch_clouds.size();i++)
		{
			Normalt nor;
			pcl::PointNormal pn=normal_cloud->at(i);
			nor.normal_x = pn.normal_x;
			nor.normal_y = pn.normal_y;
			nor.normal_z = pn.normal_z;
			double normalizeValue = pow(nor.normal_x,2) + pow(nor.normal_y,2) + pow(nor.normal_z,2);
			nor.normal_x /= normalizeValue;
			nor.normal_y /= normalizeValue;
			nor.normal_z /= normalizeValue;
			cPointCloudAnalysis.cBinarySeg.vecPatcNormal.push_back(nor);
		}
		
		cPointCloudAnalysis.cBinarySeg.vecPatchPoint.insert(cPointCloudAnalysis.cBinarySeg.vecPatchPoint.end(),patch_clouds.begin(),patch_clouds.end());
		cPointCloudAnalysis.cBinarySeg.clusterPatchNum.push_back(patch_clouds.size());
	}
 
	return cluster_points.size();
}

void CameraParaDlg::runOverSegmentation()
{
  Visualizer vs;
  vs.viewer->removeAllPointClouds();
  vs.viewer->removeCoordinateSystem();
  vs.viewer->setBackgroundColor(0,0,0);

  PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);
  loadPointCloud_normal_ply("data/GraphUpdate/init.ply", cloud);

  /******************detect table************************/
  PointCloudPtr_RGB_NORMAL tabletopCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr_RGB_NORMAL planeCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr rect_cloud(new PointCloud());
  PointCloudPtr_RGB_NORMAL remainingCloud(new PointCloud_RGB_NORMAL());
  pcl::ModelCoefficients coefficients;

  detect_table(cloud, coefficients, planeCloud, rect_cloud, remainingCloud);

  PointCloudPtr_RGB pc(new PointCloud_RGB);

  for(int i=0;i<planeCloud->size();i++){
    Point_RGB pr;
    pr.x=planeCloud->at(i).x;
    pr.y=planeCloud->at(i).y;
    pr.z=planeCloud->at(i).z;
    pr.r=planeCloud->at(i).r;
    pr.g=planeCloud->at(i).g;
    pr.b=planeCloud->at(i).b;
    pc->push_back(pr);
  }

  vs.viewer->addPointCloud (pc, "table_cloud");


  Eigen::Matrix4f matrix_transform;
  Eigen::Matrix4f matrix_transform_r;

  getTemTransformMatrix(coefficients, matrix_transform, matrix_transform_r);

  getCloudOnTable(remainingCloud, rect_cloud, matrix_transform, matrix_transform_r, tabletopCloud);

  float voxel_resolution = 0.004f;
  float seed_resolution = 0.06f;
  float color_importance = 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  /******************Euclidean Cluster Extraction************************/
  std::vector<MyPointCloud_RGB_NORMAL> cluster_points;

  object_seg_ECE(tabletopCloud, cluster_points);

  for(int i=0;i<cluster_points.size();i++){
    if(cluster_points.at(i).mypoints.size()<200){
      continue;
    }

    PointCloudT::Ptr colored_cloud(new PointCloudT);
    vector<MyPointCloud_RGB_NORMAL> patch_clouds;
    PointNCloudT::Ptr normal_cloud(new PointNCloudT);

    PointCloudPtr_RGB_NORMAL ct(new PointCloud_RGB_NORMAL);
    MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(cluster_points.at(i), ct);

    VCCS_over_segmentation(ct,voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance,patch_clouds,colored_cloud,normal_cloud);

    std::stringstream str;
    str<<"colored_voxel_cloud"<<i;
    std::string id_pc=str.str();

    vs.viewer->addPointCloud (colored_cloud, id_pc);

	//add normal, point cloud, cluster patch num
	for(int i=0;i<patch_clouds.size();i++)
	{
		Normalt nor;
		pcl::PointNormal pn=normal_cloud->at(i);
		nor.normal_x = pn.normal_x;
		nor.normal_y = pn.normal_y;
		nor.normal_z = pn.normal_z;
		double normalizeValue = pow(nor.normal_x,2) + pow(nor.normal_y,2) + pow(nor.normal_z,2);
		nor.normal_x /= normalizeValue;
		nor.normal_y /= normalizeValue;
		nor.normal_z /= normalizeValue;
		cPointCloudAnalysis.cBinarySeg.vecPatcNormal.push_back(nor);
	}

	cPointCloudAnalysis.cBinarySeg.vecPatchPoint.insert(cPointCloudAnalysis.cBinarySeg.vecPatchPoint.end(),patch_clouds.begin(),patch_clouds.end());
	cPointCloudAnalysis.cBinarySeg.clusterPatchNum.push_back(patch_clouds.size());
  }

  vs.show();

}

void CameraParaDlg::runSceneSegmentation(){
  Visualizer vs;
  vs.viewer->removeAllPointClouds();
  vs.viewer->removeCoordinateSystem();
  vs.viewer->setBackgroundColor(0,0,0);

  PointCloudPtr_RGB cloud(new PointCloud_RGB);

  //loadPointCloud_ply("data/scene0.ply", cloud);
  //loadPointCloud_ply("data/scene1.ply", cloud);
  //loadPointCloud_ply("data/big_table_after.ply", cloud);
  loadPointCloud_ply("data/two_tables.ply", cloud);
  //loadPointCloud_ply("data/big_room.ply", cloud);

  /******************detect floor and wall************************/
  MyPointCloud_RGB floor_cloud;
  pcl::ModelCoefficients floor_coefficients;
  MyPointCloud floor_rect_cloud;
  vector<MyPointCloud_RGB> wall_clouds;
  std::vector<MyPointCloud> wall_rect_clouds;
  PointCloudPtr_RGB remained_cloud(new PointCloud_RGB);

  detect_floor_and_walls(cloud, floor_cloud, floor_coefficients, floor_rect_cloud, wall_clouds, wall_rect_clouds, remained_cloud);

  if(floor_cloud.mypoints.size()>0){
    Eigen::Matrix4f matrix_transform;
    Eigen::Matrix4f matrix_translation_r;
    Eigen::Matrix4f matrix_transform_r;
    getTemTransformMatrix(floor_coefficients, floor_rect_cloud, matrix_transform, matrix_translation_r, matrix_transform_r);

    PointCloudPtr_RGB filter_remained_cloud(new PointCloud_RGB);
    remove_outliers(remained_cloud, floor_rect_cloud, wall_rect_clouds, matrix_transform, matrix_translation_r, matrix_transform_r, filter_remained_cloud, vs);

    /******************pre-segment scene************************/
    vector<MyPointCloud_RGB> cluster_projected_pcs;
    vector<MyPointCloud_RGB> cluster_origin_pcs;
    PointCloudPtr_RGB colored_projected_pc(new PointCloud_RGB);
    PointCloudPtr_RGB colored_origin_pc(new PointCloud_RGB);
    pre_segment_scene(filter_remained_cloud, matrix_transform, matrix_translation_r, matrix_transform_r, cluster_projected_pcs, cluster_origin_pcs, colored_projected_pc, colored_origin_pc);

    /******************Set Priority for Clusters************************/
    vector<int> priority_vec;
    setPriorityforClusters(cluster_projected_pcs, cluster_origin_pcs ,priority_vec);

    PointCloudPtr_RGB colored_pc(new PointCloud_RGB);
    for(int i=priority_vec.size()-1; i>=0; i--){

      cout<<"priority_vec.at(i):"<<priority_vec.at(i)<<endl;
      PointCloudPtr_RGB cloud_tem(new PointCloud_RGB);

      MyPointCloud_RGB2PointCloud_RGB(cluster_origin_pcs.at(priority_vec.at(i)), cloud_tem);
      float r, g, b;
      getColorByValue(i*1.0, 0, (cluster_origin_pcs.size()-1)*1.0, &r, &g, &b);
      for(int j=0; j<cloud_tem->size(); j++){
        cloud_tem->at(j).r=r;
        cloud_tem->at(j).g=g;
        cloud_tem->at(j).b=b;
      }

      appendCloud_RGB(cloud_tem, colored_pc);
    }

    vs.viewer->addPointCloud(colored_pc, "c1");

    Point position;
    PointCloudPtr_RGB cl(new PointCloud_RGB);
    MyPointCloud_RGB2PointCloud_RGB(cluster_origin_pcs.at(priority_vec.at(priority_vec.size()-1)), cl);
    getRobotPosition1(cl, wall_rect_clouds, matrix_transform, matrix_translation_r, matrix_transform_r, position, vs);

    vs.show();
  }
}

