//
// Created by kangd on 14.11.17.
//

#ifndef RAISIM_PEROBJECTCONTACT_HPP
#define RAISIM_PEROBJECTCONTACT_HPP

#include "raisim/math.hpp"
#include "Contact.hpp"

namespace raisim {
namespace contact {

class PerObjectContactList {
  /// Each Objects has list of Contact
 public:

  PerObjectContactList() {
    contacts_.reserve(10);
  }

  void addContact(Contact& contact) {
    contacts_.push_back(contact);
  }

  void setDelassusAndTauStar(const std::vector<std::pair<std::vector<Mat<3, 3>>, Vec<3>>> &delassusAndTauStar) {
    delassusAndTauStar_ = delassusAndTauStar;
  }

  void clearContacts() {
    contacts_.clear();
  }

  unsigned long int getNumContacts() const {
    return contacts_.size();
  }

  Contact &getContactAt(int index) {
    return contacts_[index];
  }

  const std::vector<Contact> &getContacts() const {
    return contacts_;
  }

  std::vector<Contact> &getContacts() {
    return contacts_;
  }

  void saveImpulsesForWarmStart() {
    contactN_ = contacts_.size();
    impulsesSaved_.resize(contactN_);
    localIdxSaved_.resize(contactN_);
    objIdxSaved_.resize(contactN_);

    for(size_t i=0; i<contactN_; i++)
      if(contacts_[i].isObjectA()) {
        matTransposevecmul(contacts_[i].getContactFrame(), *contacts_[i].getImpulse(), impulsesSaved_[i]);
        localIdxSaved_[i] = contacts_[i].getlocalBodyIndex();
        objIdxSaved_[i] = contacts_[i].getPairObjectIndex();
      } else {
        objIdxSaved_[i] = -1;
      }
  }

  void warmStart() {
    if(contactN_ != contacts_.size()){
      for (auto &pro: contacts_)
        pro.getImpulse()->setZero();
      return;
    }

    for(size_t i=0; i<contactN_; i++)
      if(contacts_[i].isObjectA()) {
          if(localIdxSaved_[i] == contacts_[i].getlocalBodyIndex() &&
              objIdxSaved_[i] == contacts_[i].getPairObjectIndex()) {
            matvecmul(contacts_[i].getContactFrame(), impulsesSaved_[i], *contacts_[i].getImpulse());
          } else {
            contacts_[i].getImpulse()->setZero();
          }
        }
  }

  std::vector<std::pair<std::vector<Mat<3, 3>>, Vec<3>>> &getDelassusAndTauStar() {
    return delassusAndTauStar_;
  }

  std::vector<double> &getImpactVel() {
    return impactVel_;
  }


 private:
  std::vector<std::pair<std::vector <raisim::Mat<3, 3>>, raisim::Vec<3>>> delassusAndTauStar_;
  std::vector<double> impactVel_;
  std::vector<Contact> contacts_;
  std::vector<Vec<3>> impulsesSaved_;
  std::vector<int> localIdxSaved_;
  std::vector<int> objIdxSaved_;
  size_t contactN_ = 0;

};

} // contact
} // raisim

#endif //RAISIM_PEROBJECTCONTACT_HPP
